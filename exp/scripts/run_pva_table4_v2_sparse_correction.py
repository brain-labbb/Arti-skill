#!/usr/bin/env python3
"""Build and verify the targeted PV-A Table 4 v2 corrective overlay.

The sealed 302,440-asset v2 release remains immutable.  This utility scans the
entire parent cohort with both the archived parent sampling core and the live
corrected core, evaluates only rows whose sampling plan or rest normalizer can
change, and publishes a sparse SQLite overlay.  Effective records are resolved
by parent ordinal, with overlay rows taking precedence over parent rows.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sqlite3
import stat
import sys
import tempfile
import time
from typing import Any, BinaryIO, Callable, Iterator, Mapping, Sequence
import zlib


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
EXP_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_pva_table4_mimic_aware_full_release as pva_adapter
import run_table4_full_release as table4


RUN_SCHEMA = "pva_table4_v2_sparse_correction_overlay_run_v1"
SELECTION_SCHEMA = "pva_table4_v2_sparse_correction_selection_v1"
OVERLAY_DB_SCHEMA = "pva_table4_v2_sparse_correction_db_v1"
SUMMARY_SCHEMA = "pva_table4_v2_sparse_correction_summary_v1"
CHECKPOINT_SCHEMA = "pva_table4_v2_sparse_correction_checkpoint_v1"
ARTIFACT_SCHEMA = "pva_table4_v2_sparse_correction_artifacts_v1"
RECEIPT_SCHEMA = "pva_table4_v2_sparse_correction_receipt_v1"
VERIFICATION_SCHEMA = "pva_table4_v2_sparse_correction_verification_v1"
EFFECTIVE_PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v2_corrected_r1"
OVERLAY_POLICY = "selected_parent_ordinal_atomic_record_and_state_group_replacement_v1"

DEFAULT_PARENT = EXP_ROOT / "runtime" / "pva_table4_mimic_aware_full_release_20260827"
DEFAULT_OUTPUT = EXP_ROOT / "runtime" / "pva_table4_v2_targeted_correction_20260828"
DEFAULT_PARENT_SOURCE_SNAPSHOT = DEFAULT_OUTPUT / "parent_source_snapshot"
PROTOCOL_DOCUMENT = EXP_ROOT / "URDF-Sim-Ready-Automatic-Evaluation.md"
AUDIT_NAME = "selection_audit.jsonl"
DATABASE_NAME = "overlay.sqlite3"

RAW_STATE_FIELDS = (
    "reset_readback_max_abs_error",
    "metric_max_penetration_m",
    "all_pair_contact_count",
    "all_pair_illegal_penetration_count",
    "all_pair_max_penetration_m",
    "non_adjacent_contact_count",
    "non_adjacent_illegal_penetration_count",
    "non_adjacent_max_penetration_m",
)


class CorrectionError(RuntimeError):
    pass


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


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


def self_hash(value: Mapping[str, Any], field: str) -> str:
    payload = dict(value)
    payload.pop(field, None)
    return canonical_sha256(payload)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise CorrectionError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise CorrectionError(f"{label} is not a JSON object: {path}")
    return value


def require_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != self_hash(value, field):
        raise CorrectionError(f"{label} self-hash mismatch")


def _atomic_bytes(path: Path, payload: bytes) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise CorrectionError(f"refusing to replace symlink: {path}")
    descriptor, name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, value: Any) -> None:
    _atomic_bytes(
        path,
        (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n").encode(
            "utf-8"
        ),
    )


def file_binding(path: Path, label: str) -> dict[str, Any]:
    path = Path(path).resolve(strict=True)
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise CorrectionError(f"{label} is not a regular non-symlink file: {path}")
    return {"path": str(path), "bytes": int(info.st_size), "sha256": sha256_file(path)}


def verify_binding(binding: Mapping[str, Any], label: str) -> Path:
    path = Path(str(binding.get("path", "")))
    if not path.is_absolute():
        raise CorrectionError(f"{label} path is not absolute")
    observed = file_binding(path, label)
    if observed != dict(binding):
        raise CorrectionError(f"{label} file binding mismatch")
    return path.resolve(strict=True)


class CanonicalListHasher:
    def __init__(self) -> None:
        self._digest = hashlib.sha256()
        self._digest.update(b"[")
        self._count = 0

    def add(self, value: Any) -> None:
        if self._count:
            self._digest.update(b",")
        self._digest.update(canonical_text(value).encode("utf-8"))
        self._count += 1

    def finish(self) -> tuple[str, int]:
        self._digest.update(b"]")
        return self._digest.hexdigest(), self._count


def _sidecar_snapshot(database: Path) -> dict[str, tuple[int, int, int, int] | None]:
    snapshot: dict[str, tuple[int, int, int, int] | None] = {}
    for suffix in ("-wal", "-shm", "-journal"):
        sidecar = Path(f"{database}{suffix}")
        try:
            info = sidecar.lstat()
        except FileNotFoundError:
            snapshot[suffix] = None
            continue
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
            raise CorrectionError(f"sealed SQLite sidecar is unsafe: {sidecar}")
        if suffix in {"-wal", "-journal"} and info.st_size != 0:
            raise CorrectionError(f"sealed SQLite sidecar is non-empty: {sidecar}")
        snapshot[suffix] = (
            int(info.st_dev),
            int(info.st_ino),
            int(info.st_size),
            int(info.st_mtime_ns),
        )
    return snapshot


def verify_sealed_database(path: Path, expected_sha256: str) -> Path:
    path = Path(path)
    try:
        before = path.lstat()
    except FileNotFoundError as error:
        raise CorrectionError(f"sealed SQLite database is missing: {path}") from error
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise CorrectionError(f"sealed SQLite database is unsafe: {path}")
    before_sidecars = _sidecar_snapshot(path)
    if sha256_file(path) != expected_sha256:
        raise CorrectionError(f"sealed SQLite SHA-256 mismatch: {path}")
    after = path.lstat()
    after_sidecars = _sidecar_snapshot(path)
    identity = lambda info: (
        int(info.st_dev),
        int(info.st_ino),
        int(info.st_size),
        int(info.st_mtime_ns),
    )
    if identity(before) != identity(after) or before_sidecars != after_sidecars:
        raise CorrectionError(f"sealed SQLite changed during verification: {path}")
    return path.resolve(strict=True)


def connect_immutable(path: Path, expected_sha256: str, *, verify_hash: bool = True) -> sqlite3.Connection:
    resolved = (
        verify_sealed_database(path, expected_sha256)
        if verify_hash
        else Path(path).resolve(strict=True)
    )
    connection = sqlite3.connect(
        f"{resolved.as_uri()}?mode=ro&immutable=1", uri=True, timeout=120.0
    )
    connection.execute("PRAGMA query_only=ON")
    if int(connection.execute("PRAGMA query_only").fetchone()[0]) != 1:
        connection.close()
        raise CorrectionError("failed to enforce immutable query-only SQLite")
    return connection


def connect_overlay(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(path, timeout=120.0)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute("PRAGMA temp_store=MEMORY")
    connection.execute("PRAGMA foreign_keys=ON")
    return connection


def _decoded_meta(connection: sqlite3.Connection) -> dict[str, Any]:
    return {
        str(key): json.loads(value)
        for key, value in connection.execute("SELECT key, value FROM meta")
    }


def _set_meta(connection: sqlite3.Connection, values: Mapping[str, Any]) -> None:
    connection.executemany(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        ((str(key), canonical_text(value)) for key, value in values.items()),
    )


def _create_overlay_schema(connection: sqlite3.Connection) -> None:
    connection.executescript(
        """
        CREATE TABLE meta(
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        CREATE TABLE selection(
            selection_index INTEGER PRIMARY KEY,
            parent_ordinal INTEGER NOT NULL UNIQUE,
            asset_id TEXT NOT NULL UNIQUE,
            source_row_sha256 TEXT NOT NULL,
            reasons_json TEXT NOT NULL,
            old_execution_sig_sha256 TEXT NOT NULL,
            new_execution_sig_sha256 TEXT NOT NULL,
            old_scale_pose_sha256 TEXT NOT NULL,
            new_scale_pose_sha256 TEXT NOT NULL,
            new_plan_sha256 TEXT,
            new_independent_dof INTEGER NOT NULL,
            new_fixed_root_count INTEGER NOT NULL,
            audit_json TEXT NOT NULL,
            row_content_sha256 TEXT NOT NULL
        );
        CREATE TABLE results(
            parent_ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            record_json TEXT NOT NULL,
            states_zlib BLOB NOT NULL,
            state_count INTEGER NOT NULL CHECK(state_count >= 0),
            worker_status TEXT NOT NULL,
            worker_seconds REAL NOT NULL CHECK(worker_seconds >= 0),
            completed_at_utc TEXT NOT NULL,
            FOREIGN KEY(parent_ordinal) REFERENCES selection(parent_ordinal)
        );
        """
    )


def _load_module(path: Path, name: str) -> Any:
    path = Path(path).resolve(strict=True)
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise CorrectionError(f"cannot load Python source snapshot: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _stable_number(value: Any) -> Any:
    if not isinstance(value, (int, float)):
        return value
    number = float(value)
    if math.isnan(number):
        return "NaN"
    if math.isinf(number):
        return "+Infinity" if number > 0 else "-Infinity"
    return number


def _joint_interval_signature(row: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "name": str(row.get("name", "")),
        "lower": _stable_number(row.get("sampling_lower", row.get("lower"))),
        "upper": _stable_number(row.get("sampling_upper", row.get("upper"))),
        "range_evaluable": bool(
            row.get("sampling_range_evaluable", row.get("range_evaluable"))
        ),
        "zero_baseline_evaluable": bool(row.get("zero_baseline_evaluable", True)),
    }


def _plan_execution_signature(core: Any, plan: Mapping[str, Any]) -> dict[str, Any]:
    independent_rows = list(plan.get("independent_joints", []))
    fixed_values = {
        int(index): float(value)
        for index, value in dict(plan.get("fixed_root_values", {})).items()
    }
    root_positions = {
        int(index): int(position)
        for index, position in dict(plan.get("root_position_by_index", {})).items()
    }
    bindings = []
    for row in plan.get("binding_rows", []):
        root_index = int(row["root_index"])
        coefficient = float(row["multiplier"])
        offset = float(row["offset"])
        if root_index in fixed_values:
            binding = {
                "joint_name": str(row["joint_name"]),
                "constant": coefficient * fixed_values[root_index] + offset,
            }
        else:
            if root_index not in root_positions:
                raise CorrectionError(f"sampling plan lacks root position {root_index}")
            binding = {
                "joint_name": str(row["joint_name"]),
                "variable": str(row["root_name"]),
                "coefficient": coefficient,
                "offset": offset,
            }
        bindings.append(binding)
    independent_count = int(plan["independent_dof_count"])
    rest_values = core.expand_joint_values(plan, [0.0] * independent_count)
    single_rows = [
        _joint_interval_signature(row)
        for row in independent_rows
        if bool(row.get("sampling_range_evaluable", row.get("range_evaluable")))
    ]
    sobol_enabled = bool(independent_rows) and all(
        bool(row.get("sampling_range_evaluable", row.get("range_evaluable")))
        for row in independent_rows
    )
    return {
        "joint_names": [str(row.get("name", "")) for row in plan.get("joints", [])],
        "bindings": bindings,
        "independent_intervals": [
            _joint_interval_signature(row) for row in independent_rows
        ],
        "fixed_roots": [
            {"root_index": index, "value": fixed_values[index]}
            for index in sorted(fixed_values)
        ],
        "rest_joint_values_sha256": canonical_sha256(rest_values),
        "single": {
            "samples_per_joint": int(table4.SINGLE_SAMPLES),
            "rows": single_rows,
        },
        "sobol": {
            "enabled": sobol_enabled,
            "dimension": independent_count if sobol_enabled else 0,
            "samples": int(table4.SOBOL_SAMPLES) if independent_count else 0,
            "seed": int(table4.SOBOL_SEED),
            "rows": (
                [_joint_interval_signature(row) for row in independent_rows]
                if sobol_enabled
                else []
            ),
        },
        "expected": {
            "rest": 1,
            "single": int(table4.SINGLE_SAMPLES) * independent_count,
            "sobol": int(table4.SOBOL_SAMPLES) if independent_count else 0,
        },
        "independent_dof_count": independent_count,
        "range_evaluable_independent_dof_count": int(
            plan["range_evaluable_independent_dof_count"]
        ),
        "mimic_joint_count": int(plan["mimic_joint_count"]),
        "fixed_root_joint_count": len(fixed_values),
        "plan_sha256": str(plan["plan_sha256"]),
    }


def _expected_parent_input_identity(
    row: Mapping[str, Any], plan: Mapping[str, Any]
) -> str:
    independent = int(plan["independent_dof_count"])
    values = {
        "dataset": "pva",
        "dataset_id": str(row["asset_id"]),
        "category": str(row.get("raw_category", row.get("category", ""))),
        "urdf_path": str(Path(str(row["primary_urdf_path"])).resolve()),
        "primary_urdf_relative_path": str(
            row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
        "expected_movable_joints": int(row.get("joint_count", 0)),
        "package_binding_sha256": row.get("package_binding_sha256"),
        "sampling_protocol": table4.SAMPLING_PROTOCOL_V2,
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": int(
            plan["range_evaluable_independent_dof_count"]
        ),
        "mimic_joint_count": int(plan["mimic_joint_count"]),
        "joint_sampling_plan_sha256": str(plan["plan_sha256"]),
        "single_state_expected": int(table4.SINGLE_SAMPLES) * independent,
        "sobol_state_expected": int(table4.SOBOL_SAMPLES) if independent else 0,
    }
    return canonical_sha256(values)


def _backend_capacity_issue(record: Mapping[str, Any], joint_count: int) -> bool:
    if joint_count <= 121 or str(record.get("status", "")) != "error":
        return False
    issues = record.get("issues")
    if not isinstance(issues, list) or not issues:
        return False
    text = "\n".join(str(value) for value in issues)
    return "getJointState failed" in text or "reset/readback error" in text


def _mapped_urdf(row: Mapping[str, Any], binding: Mapping[str, Any]) -> Path:
    logical_root = Path(str(binding["logical_root"]))
    physical_root = Path(str(binding["physical_root"]))
    logical_urdf = Path(str(row["primary_urdf_path"]))
    if not logical_root.is_absolute() or not physical_root.is_absolute() or not logical_urdf.is_absolute():
        raise CorrectionError("package-root mapping contains a non-absolute path")
    try:
        relative = logical_urdf.relative_to(logical_root)
    except ValueError as error:
        raise CorrectionError(f"source URDF escapes logical root: {logical_urdf}") from error
    candidate = physical_root / relative
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(physical_root.resolve(strict=True))
    except ValueError as error:
        raise CorrectionError(f"mapped URDF escapes physical root: {resolved}") from error
    if not resolved.is_file() or resolved.is_symlink():
        raise CorrectionError(f"mapped URDF is unsafe: {resolved}")
    return resolved


def _source_snapshot_bindings(
    snapshot: Path, parent_manifest: Mapping[str, Any]
) -> dict[str, dict[str, Any]]:
    expected = {
        "adapter": (
            "run_pva_table4_mimic_aware_full_release.py",
            parent_manifest["source_hashes"]["adapter"],
        ),
        "runner": (
            "run_table4_full_release.py",
            parent_manifest["source_hashes"]["table4_runner"],
        ),
        "core": (
            "run_urdf_table4_partnet_mobility.py",
            parent_manifest["source_hashes"]["table4_core"],
        ),
    }
    bindings: dict[str, dict[str, Any]] = {}
    for name, (filename, digest) in expected.items():
        binding = file_binding(Path(snapshot) / filename, f"parent {name} snapshot")
        if binding["sha256"] != digest:
            raise CorrectionError(f"parent {name} snapshot does not match parent manifest")
        bindings[name] = binding
    return bindings


def _parent_context(parent: Path, snapshot: Path, *, verify_databases: bool) -> dict[str, Any]:
    parent = Path(parent).resolve(strict=True)
    receipt_path = parent / "full_release_receipt.json"
    manifest_path = parent / "manifest.json"
    artifact_path = parent / "artifact_manifest.json"
    summary_path = parent / "summary.json"
    receipt = load_json(receipt_path, "parent receipt")
    manifest = load_json(manifest_path, "parent manifest")
    artifact = load_json(artifact_path, "parent artifact manifest")
    summary = load_json(summary_path, "parent summary")
    require_self_hash(receipt, "receipt_content_sha256", "parent receipt")
    require_self_hash(manifest, "manifest_content_sha256", "parent manifest")
    require_self_hash(artifact, "artifact_manifest_content_sha256", "parent artifact manifest")
    require_self_hash(summary, "summary_content_sha256", "parent summary")
    bindings = {
        "receipt": file_binding(receipt_path, "parent receipt"),
        "manifest": file_binding(manifest_path, "parent manifest"),
        "artifact_manifest": file_binding(artifact_path, "parent artifact manifest"),
        "summary": file_binding(summary_path, "parent summary"),
    }
    for field, name in (
        ("manifest_sha256", "manifest"),
        ("artifact_manifest_sha256", "artifact_manifest"),
        ("summary_sha256", "summary"),
    ):
        if receipt.get(field) != bindings[name]["sha256"]:
            raise CorrectionError(f"parent receipt binding mismatch: {field}")
    result_database = parent / str(receipt["result_database"])
    expected_result_hash = str(receipt["result_database_sha256"])
    if verify_databases:
        verify_sealed_database(result_database, expected_result_hash)
    artifact_rows = {
        str(row.get("path")): row
        for row in artifact.get("artifacts", [])
        if isinstance(row, Mapping)
    }
    result_artifact = artifact_rows.get(str(receipt["result_database"]))
    if (
        not isinstance(result_artifact, Mapping)
        or result_artifact.get("sha256") != expected_result_hash
        or int(result_artifact.get("bytes", -1)) != result_database.stat().st_size
    ):
        raise CorrectionError("parent artifact manifest result database binding mismatch")
    source = receipt.get("source")
    if not isinstance(source, Mapping):
        raise CorrectionError("parent receipt lacks source binding")
    source_receipt_path = Path(str(source["source_receipt"])).resolve(strict=True)
    source_receipt = load_json(source_receipt_path, "source receipt")
    require_self_hash(source_receipt, "receipt_content_sha256", "source receipt")
    if sha256_file(source_receipt_path) != source.get("source_receipt_sha256"):
        raise CorrectionError("source receipt file binding mismatch")
    source_database = source_receipt_path.parent / str(source_receipt["result_database"])
    source_database_hash = str(source_receipt["result_database_sha256"])
    if source.get("source_result_database_declared_sha256") != source_database_hash:
        raise CorrectionError("source database digest declaration mismatch")
    if verify_databases:
        verify_sealed_database(source_database, source_database_hash)
    roster_path = Path(str(source_receipt["roster_manifest"])).resolve(strict=True)
    roster = load_json(roster_path, "source roster")
    require_self_hash(roster, "manifest_content_sha256", "source roster")
    if sha256_file(roster_path) != source_receipt.get("roster_manifest_sha256"):
        raise CorrectionError("source roster file binding mismatch")
    snapshot_bindings = _source_snapshot_bindings(snapshot, manifest)
    package_binding = manifest.get("package_root_binding")
    if not isinstance(package_binding, Mapping):
        raise CorrectionError("parent manifest lacks package-root binding")
    if package_binding.get("binding_content_sha256") != self_hash(
        package_binding, "binding_content_sha256"
    ):
        raise CorrectionError("parent package-root binding self-hash mismatch")
    if int(receipt["N_eval"]) != int(manifest["N_eval"]):
        raise CorrectionError("parent N_eval binding mismatch")
    parent_db = connect_immutable(result_database, expected_result_hash, verify_hash=False)
    source_db = connect_immutable(source_database, source_database_hash, verify_hash=False)
    try:
        parent_meta = _decoded_meta(parent_db)
        source_meta = _decoded_meta(source_db)
        for key, expected in {
            "schema_version": "pva_table4_mimic_aware_results_db_v1",
            "N_eval": int(receipt["N_eval"]),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        }.items():
            if parent_meta.get(key) != expected:
                raise CorrectionError(f"parent result database metadata mismatch: {key}")
        for key, expected in {
            "schema_version": "pva_table1234_results_db_v1",
            "asset_import_state": "COMPLETE",
            "selected_n": int(receipt["N_eval"]),
            "selected_j": int(receipt["J_eval"]),
            "roster_manifest_sha256": source_receipt["roster_manifest_sha256"],
            "roster_manifest_content_sha256": source_receipt[
                "roster_manifest_content_sha256"
            ],
        }.items():
            if source_meta.get(key) != expected:
                raise CorrectionError(f"source roster database metadata mismatch: {key}")
    finally:
        parent_db.close()
        source_db.close()
    return {
        "root": parent,
        "receipt": receipt,
        "manifest": manifest,
        "artifact": artifact,
        "summary": summary,
        "bindings": bindings,
        "result_database": result_database.resolve(strict=True),
        "result_database_sha256": expected_result_hash,
        "source_receipt": source_receipt,
        "source_receipt_path": source_receipt_path,
        "source_database": source_database.resolve(strict=True),
        "source_database_sha256": source_database_hash,
        "roster": roster,
        "roster_path": roster_path,
        "package_binding": dict(package_binding),
        "snapshot_bindings": snapshot_bindings,
    }


def _selection_detail(
    *,
    ordinal: int,
    asset_id: str,
    source_row_sha256: str,
    parent_record: Mapping[str, Any],
    parent_record_json: str,
    joints: Sequence[Mapping[str, Any]],
    old_plan: Mapping[str, Any],
    new_plan: Mapping[str, Any],
    old_signature: Mapping[str, Any],
    new_signature: Mapping[str, Any],
    include_backend_capacity_errors: bool,
) -> dict[str, Any]:
    # Normalization is deliberately kept in its own pose hash.  A changed
    # expanded-rest pose must not be mistaken for a changed sampling plan.
    old_execution_signature = {
        key: value
        for key, value in old_signature.items()
        if key != "rest_joint_values_sha256"
    }
    new_execution_signature = {
        key: value
        for key, value in new_signature.items()
        if key != "rest_joint_values_sha256"
    }
    old_signature_hash = canonical_sha256(old_execution_signature)
    new_signature_hash = canonical_sha256(new_execution_signature)
    old_scale_pose_hash = canonical_sha256([0.0] * len(joints))
    new_rest = str(new_signature["rest_joint_values_sha256"])
    reasons: list[str] = []
    recognized: list[str] = []
    if old_signature_hash != new_signature_hash:
        old_names = [str(row["name"]) for row in old_plan["independent_joints"]]
        new_names = [str(row["name"]) for row in new_plan["independent_joints"]]
        removed = [name for name in old_names if name not in set(new_names)]
        by_name = {str(row["name"]): row for row in joints}
        valid_removed = bool(removed) and all(
            by_name[name].get("mimic") is None
            and isinstance(by_name[name].get("lower"), (int, float))
            and isinstance(by_name[name].get("upper"), (int, float))
            and abs(float(by_name[name]["lower"])) <= float(new_plan.get("zero_width_tolerance", 1e-12))
            and abs(float(by_name[name]["upper"])) <= float(new_plan.get("zero_width_tolerance", 1e-12))
            for name in removed
        )
        if (
            not valid_removed
            or any(name not in old_names for name in new_names)
            or len(removed) != int(new_plan.get("fixed_root_joint_count", 0))
        ):
            raise CorrectionError(
                f"unexpected old/new sampling-plan change at {ordinal}: {asset_id}"
            )
        reasons.append("zero_width_fixed_root")
    if old_scale_pose_hash != new_rest:
        reasons.append("expanded_rest_normalizer")
    joint_count = int(parent_record.get("expected_movable_joints", len(joints)))
    capacity = _backend_capacity_issue(parent_record, joint_count)
    if capacity:
        recognized.append("recognized_parent_backend_capacity_retained")
        if reasons:
            raise CorrectionError(
                f"backend-capacity row overlaps corrective selection at {ordinal}: {asset_id}"
            )
        if include_backend_capacity_errors:
            reasons.append("backend_capacity_retry")
    detail: dict[str, Any] = {
        "schema_version": SELECTION_SCHEMA,
        "parent_ordinal": ordinal,
        "asset_id": asset_id,
        "source_row_sha256": source_row_sha256,
        "parent_record_sha256": hashlib.sha256(parent_record_json.encode("utf-8")).hexdigest(),
        "parent_state_records_sha256": parent_record.get("state_records_sha256"),
        "selected": bool(reasons),
        "selection_reasons": reasons,
        "recognized_reasons": recognized,
        "old_execution_sig_sha256": old_signature_hash,
        "new_execution_sig_sha256": new_signature_hash,
        "old_scale_pose_sha256": old_scale_pose_hash,
        "new_scale_pose_sha256": new_rest,
        "old_plan_sha256": str(old_plan["plan_sha256"]),
        "new_plan_sha256": str(new_plan["plan_sha256"]),
        "old_independent_dof": int(old_plan["independent_dof_count"]),
        "new_independent_dof": int(new_plan["independent_dof_count"]),
        "new_fixed_root_count": int(new_plan.get("fixed_root_joint_count", 0)),
        "parent_unexecuted_state_count": sum(
            int(parent_record.get(f"{phase}_state_expected", 0) or 0)
            - int(parent_record.get(f"{phase}_state_executed", 0) or 0)
            for phase in ("rest", "single", "sobol")
        ),
    }
    detail["row_content_sha256"] = self_hash(detail, "row_content_sha256")
    return detail


def _scan_selection(
    context: Mapping[str, Any],
    old_core: Any,
    new_core: Any,
    *,
    include_backend_capacity_errors: bool,
    audit_stream: BinaryIO | None = None,
    expected_audit_stream: BinaryIO | None = None,
    selected_callback: Callable[[int, Mapping[str, Any]], None] | None = None,
) -> dict[str, Any]:
    parent = connect_immutable(
        context["result_database"], context["result_database_sha256"], verify_hash=False
    )
    source = connect_immutable(
        context["source_database"], context["source_database_sha256"], verify_hash=False
    )
    selected_hasher = CanonicalListHasher()
    unselected_hasher = CanonicalListHasher()
    audit_hasher = hashlib.sha256()
    reason_counts: Counter[str] = Counter()
    recognized_counts: Counter[str] = Counter()
    recognized_unexecuted: Counter[str] = Counter()
    totals: Counter[str] = Counter()
    selected_index = 0
    corrective_selected = 0
    observed = 0
    n_eval = int(context["receipt"]["N_eval"])
    parent_cursor = parent.execute(
        "SELECT ordinal, asset_id, record_json, state_count FROM results ORDER BY ordinal"
    )
    source_cursor = source.execute(
        "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
        "FROM assets ORDER BY ordinal"
    )
    try:
        for expected in range(n_eval):
            parent_values = parent_cursor.fetchone()
            source_values = source_cursor.fetchone()
            if parent_values is None or source_values is None:
                raise CorrectionError(f"parent/source database ended at ordinal {expected}")
            parent_ordinal, parent_id, record_json, state_count = parent_values
            source_ordinal, source_id, category, joint_count, row_hash, row_json = source_values
            if int(parent_ordinal) != expected or int(source_ordinal) != expected:
                raise CorrectionError(f"non-contiguous parent/source ordinal at {expected}")
            if str(parent_id) != str(source_id):
                raise CorrectionError(f"parent/source asset mismatch at {expected}")
            row = json.loads(row_json)
            record = json.loads(record_json)
            if not isinstance(row, dict) or not isinstance(record, dict):
                raise CorrectionError(f"non-object source/record at {expected}")
            if canonical_sha256(row) != str(row_hash):
                raise CorrectionError(f"source row hash mismatch at {expected}")
            if (
                int(row.get("ordinal", -1)) != expected
                or str(row.get("asset_id")) != str(source_id)
                or int(row.get("joint_count", -1)) != int(joint_count)
                or str(row.get("raw_category", row.get("category", ""))) != str(category)
            ):
                raise CorrectionError(f"source row binding mismatch at {expected}")
            if (
                int(record.get("order", -1)) != expected
                or str(record.get("dataset_id")) != str(source_id)
                or int(record.get("state_records_count", state_count)) != int(state_count)
            ):
                raise CorrectionError(f"parent record binding mismatch at {expected}")
            urdf = _mapped_urdf(row, context["package_binding"])
            if sha256_file(urdf) != str(row.get("primary_urdf_sha256", "")):
                raise CorrectionError(f"source URDF hash drift at {expected}: {source_id}")
            try:
                old_joints = old_core.parse_urdf_joints(urdf)
                new_joints = new_core.parse_urdf_joints(urdf)
                if canonical_text(old_joints) != canonical_text(new_joints):
                    raise CorrectionError(f"old/new URDF parser drift at {expected}: {source_id}")
                if len(old_joints) != int(joint_count):
                    raise CorrectionError(f"parsed joint-count drift at {expected}: {source_id}")
                old_plan = old_core.compile_joint_sampling_plan(old_joints)
                new_plan = new_core.compile_joint_sampling_plan(new_joints)
            except CorrectionError:
                raise
            except BaseException as error:
                raise CorrectionError(
                    f"sampling-plan compile failed at {expected}: {source_id}: {error}"
                ) from error
            expected_parent = {
                "independent_dof_count": int(old_plan["independent_dof_count"]),
                "range_evaluable_independent_dof_count": int(
                    old_plan["range_evaluable_independent_dof_count"]
                ),
                "mimic_joint_count": int(old_plan["mimic_joint_count"]),
                "joint_sampling_plan_sha256": str(old_plan["plan_sha256"]),
                "single_state_expected": int(table4.SINGLE_SAMPLES)
                * int(old_plan["independent_dof_count"]),
                "sobol_state_expected": (
                    int(table4.SOBOL_SAMPLES)
                    if int(old_plan["independent_dof_count"])
                    else 0
                ),
            }
            for field, value in expected_parent.items():
                if record.get(field) != value:
                    raise CorrectionError(
                        f"parent record sampling metadata mismatch at {expected}: {field}"
                    )
            if record.get("input_identity_sha256") != _expected_parent_input_identity(
                row, old_plan
            ):
                raise CorrectionError(f"parent input identity mismatch at {expected}")
            old_signature = _plan_execution_signature(old_core, old_plan)
            new_signature = _plan_execution_signature(new_core, new_plan)
            totals["old_independent_dof"] += int(old_plan["independent_dof_count"])
            totals["new_independent_dof"] += int(new_plan["independent_dof_count"])
            totals["old_range_evaluable_independent_dof"] += int(
                old_plan["range_evaluable_independent_dof_count"]
            )
            totals["new_range_evaluable_independent_dof"] += int(
                new_plan["range_evaluable_independent_dof_count"]
            )
            totals["mimic_joint_count"] += int(new_plan["mimic_joint_count"])
            totals["fixed_root_joint_count"] += int(
                new_plan.get("fixed_root_joint_count", 0)
            )
            totals["new_single_states_expected"] += int(
                new_signature["expected"]["single"]
            )
            totals["new_sobol_states_expected"] += int(
                new_signature["expected"]["sobol"]
            )
            detail = _selection_detail(
                ordinal=expected,
                asset_id=str(source_id),
                source_row_sha256=str(row_hash),
                parent_record=record,
                parent_record_json=str(record_json),
                joints=old_joints,
                old_plan=old_plan,
                new_plan=new_plan,
                old_signature=old_signature,
                new_signature=new_signature,
                include_backend_capacity_errors=include_backend_capacity_errors,
            )
            line = (canonical_text(detail) + "\n").encode("utf-8")
            audit_hasher.update(line)
            if audit_stream is not None:
                audit_stream.write(line)
            if expected_audit_stream is not None:
                wanted = expected_audit_stream.readline()
                if wanted != line:
                    raise CorrectionError(f"selection audit drift at ordinal {expected}")
            if detail["selected"]:
                selection_identity = {
                    "parent_ordinal": expected,
                    "asset_id": str(source_id),
                    "source_row_sha256": str(row_hash),
                    "selection_reasons": detail["selection_reasons"],
                }
                selected_hasher.add(selection_identity)
                reason_counts.update(str(value) for value in detail["selection_reasons"])
                if selected_callback is not None:
                    selected_callback(selected_index, detail)
                if any(
                    reason != "backend_capacity_retry"
                    for reason in detail["selection_reasons"]
                ):
                    corrective_selected += 1
                selected_index += 1
            else:
                unselected_hasher.add(
                    {
                        "parent_ordinal": expected,
                        "asset_id": str(source_id),
                        "source_row_sha256": str(row_hash),
                        "old_execution_sig_sha256": detail[
                            "old_execution_sig_sha256"
                        ],
                        "new_execution_sig_sha256": detail[
                            "new_execution_sig_sha256"
                        ],
                        "old_scale_pose_sha256": detail["old_scale_pose_sha256"],
                        "new_scale_pose_sha256": detail["new_scale_pose_sha256"],
                    }
                )
            for reason in detail["recognized_reasons"]:
                recognized_counts[str(reason)] += 1
                recognized_unexecuted[str(reason)] += int(
                    detail["parent_unexecuted_state_count"]
                )
            observed += 1
        if parent_cursor.fetchone() is not None or source_cursor.fetchone() is not None:
            raise CorrectionError("parent/source database contains rows beyond N_eval")
        if expected_audit_stream is not None and expected_audit_stream.read(1):
            raise CorrectionError("selection audit contains trailing rows")
    finally:
        parent.close()
        source.close()
    selected_hash, selected_count = selected_hasher.finish()
    unselected_hash, unselected_count = unselected_hasher.finish()
    return {
        "scanned_asset_count": observed,
        "selected_asset_count": selected_count,
        "corrective_selected_asset_count": corrective_selected,
        "unselected_asset_count": unselected_count,
        "selected_assets_sha256": selected_hash,
        "unselected_parity_sha256": unselected_hash,
        "selection_audit_sha256": audit_hasher.hexdigest(),
        "reason_counts": dict(sorted(reason_counts.items())),
        "recognized_reason_counts": dict(sorted(recognized_counts.items())),
        "recognized_unexecuted_state_counts": dict(
            sorted(recognized_unexecuted.items())
        ),
        "cohort_totals": dict(sorted(totals.items())),
    }


def _current_source_bindings() -> dict[str, dict[str, Any]]:
    return {
        "correction_script": file_binding(SCRIPT, "correction script"),
        "adapter": file_binding(Path(pva_adapter.__file__), "current PV-A adapter"),
        "runner": file_binding(Path(table4.__file__), "current Table 4 runner"),
        "core": file_binding(Path(table4.CORE_SCRIPT), "current Table 4 core"),
    }


def _database_declared_binding(path: Path, digest: str) -> dict[str, Any]:
    resolved = Path(path).resolve(strict=True)
    return {
        "path": str(resolved),
        "bytes": int(resolved.stat().st_size),
        "sha256": str(digest),
    }


def _load_cores(context: Mapping[str, Any]) -> tuple[Any, Any]:
    old_path = Path(context["snapshot_bindings"]["core"]["path"])
    new_path = Path(table4.CORE_SCRIPT)
    old_core = _load_module(old_path, f"pva_parent_core_{sha256_file(old_path)[:12]}")
    new_core = _load_module(new_path, f"pva_corrected_core_{sha256_file(new_path)[:12]}")
    return old_core, new_core


def _selection_insert(
    connection: sqlite3.Connection, selection_index: int, detail: Mapping[str, Any]
) -> None:
    connection.execute(
        "INSERT INTO selection("
        "selection_index,parent_ordinal,asset_id,source_row_sha256,reasons_json,"
        "old_execution_sig_sha256,new_execution_sig_sha256,old_scale_pose_sha256,"
        "new_scale_pose_sha256,new_plan_sha256,new_independent_dof,"
        "new_fixed_root_count,audit_json,row_content_sha256) "
        "VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
        (
            int(selection_index),
            int(detail["parent_ordinal"]),
            str(detail["asset_id"]),
            str(detail["source_row_sha256"]),
            canonical_text(detail["selection_reasons"]),
            str(detail["old_execution_sig_sha256"]),
            str(detail["new_execution_sig_sha256"]),
            str(detail["old_scale_pose_sha256"]),
            str(detail["new_scale_pose_sha256"]),
            str(detail["new_plan_sha256"]),
            int(detail["new_independent_dof"]),
            int(detail["new_fixed_root_count"]),
            canonical_text(detail),
            str(detail["row_content_sha256"]),
        ),
    )


def _prepare(args: argparse.Namespace) -> dict[str, Any]:
    if args.include_backend_capacity_errors and not args.high_capacity_runtime_attested:
        raise CorrectionError(
            "--include-backend-capacity-errors requires "
            "--high-capacity-runtime-attested"
        )
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "manifest.json"
    database_path = output / DATABASE_NAME
    audit_path = output / AUDIT_NAME
    if any(path.exists() for path in (manifest_path, database_path, audit_path)):
        raise CorrectionError(
            "prepared correction artifacts already exist; use run/finalize/verify "
            "instead of replacing them"
        )
    context = _parent_context(
        Path(args.parent),
        Path(args.parent_source_snapshot),
        verify_databases=not args.skip_input_database_rehash,
    )
    current_sources = _current_source_bindings()
    old_core, new_core = _load_cores(context)
    descriptor, audit_name = tempfile.mkstemp(
        prefix=f".{AUDIT_NAME}.", suffix=".tmp", dir=str(output)
    )
    os.close(descriptor)
    audit_temporary = Path(audit_name)
    descriptor, database_name = tempfile.mkstemp(
        prefix=f".{DATABASE_NAME}.", suffix=".tmp", dir=str(output)
    )
    os.close(descriptor)
    database_temporary = Path(database_name)
    database_temporary.unlink()
    connection: sqlite3.Connection | None = None
    try:
        connection = connect_overlay(database_temporary)
        _create_overlay_schema(connection)
        pending_inserts = 0

        def insert(selection_index: int, detail: Mapping[str, Any]) -> None:
            nonlocal pending_inserts
            _selection_insert(connection, selection_index, detail)
            pending_inserts += 1
            if pending_inserts >= int(args.batch_size):
                connection.commit()
                pending_inserts = 0

        with audit_temporary.open("wb") as audit_stream:
            scan = _scan_selection(
                context,
                old_core,
                new_core,
                include_backend_capacity_errors=bool(
                    args.include_backend_capacity_errors
                ),
                audit_stream=audit_stream,
                selected_callback=insert,
            )
            audit_stream.flush()
            os.fsync(audit_stream.fileno())
        connection.commit()
        if int(scan["corrective_selected_asset_count"]) != int(
            args.expected_correction_assets
        ):
            raise CorrectionError(
                "corrective selector count mismatch: "
                f"{scan['corrective_selected_asset_count']} != "
                f"{args.expected_correction_assets}"
            )
        if int(scan["scanned_asset_count"]) != int(context["receipt"]["N_eval"]):
            raise CorrectionError("selector did not scan the complete parent cohort")
        selected_count = int(
            connection.execute("SELECT COUNT(*) FROM selection").fetchone()[0]
        )
        if selected_count != int(scan["selected_asset_count"]):
            raise CorrectionError("selection table row count mismatch")
        prepared_at = utc_now()
        prepare_identity = {
            "schema_version": OVERLAY_DB_SCHEMA,
            "parent_receipt_content_sha256": context["receipt"][
                "receipt_content_sha256"
            ],
            "parent_result_database_sha256": context["result_database_sha256"],
            "source_result_database_sha256": context["source_database_sha256"],
            "selection_audit_sha256": scan["selection_audit_sha256"],
            "selected_assets_sha256": scan["selected_assets_sha256"],
            "selected_asset_count": selected_count,
            "include_backend_capacity_errors": bool(
                args.include_backend_capacity_errors
            ),
            "high_capacity_runtime_attested": bool(
                args.high_capacity_runtime_attested
            ),
        }
        prepare_identity["prepare_identity_sha256"] = self_hash(
            prepare_identity, "prepare_identity_sha256"
        )
        _set_meta(
            connection,
            {
                "schema_version": OVERLAY_DB_SCHEMA,
                "run_state": "PREPARED",
                "prepare_identity": prepare_identity,
                "selection_scan": scan,
                "prepared_at_utc": prepared_at,
            },
        )
        connection.commit()
        if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
            raise CorrectionError("prepared overlay database integrity check failed")
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        connection.execute("PRAGMA journal_mode=DELETE")
        connection.close()
        connection = None
        os.replace(audit_temporary, audit_path)
        os.replace(database_temporary, database_path)
        if _current_source_bindings() != current_sources:
            raise CorrectionError("correction source changed during selector scan")
        manifest: dict[str, Any] = {
            "schema_version": RUN_SCHEMA,
            "classification": "FORMAL_FULL_RELEASE_SPARSE_CORRECTION",
            "dataset": "Ours / PV-A",
            "dataset_slug": "pva",
            "effective_protocol_id": EFFECTIVE_PROTOCOL_ID,
            "overlay_policy": OVERLAY_POLICY,
            "output": str(output),
            "N_eval": int(context["receipt"]["N_eval"]),
            "J_eval": int(context["receipt"]["J_eval"]),
            "parent": {
                "root": str(context["root"]),
                "receipt_content_sha256": context["receipt"][
                    "receipt_content_sha256"
                ],
                "files": context["bindings"],
                "result_database": _database_declared_binding(
                    context["result_database"], context["result_database_sha256"]
                ),
            },
            "source_roster": {
                "receipt": file_binding(
                    context["source_receipt_path"], "source receipt"
                ),
                "database": _database_declared_binding(
                    context["source_database"], context["source_database_sha256"]
                ),
                "manifest": file_binding(context["roster_path"], "source roster"),
            },
            "parent_source_snapshot": context["snapshot_bindings"],
            "correction_sources": current_sources,
            "package_root_binding": context["package_binding"],
            "selection": {
                **scan,
                "audit": file_binding(audit_path, "selection audit"),
                "database": str(database_path),
                "expected_correction_assets": int(args.expected_correction_assets),
                "include_backend_capacity_errors": bool(
                    args.include_backend_capacity_errors
                ),
                "high_capacity_runtime_attested": bool(
                    args.high_capacity_runtime_attested
                ),
            },
            "execution": {
                "fresh_interpreter_per_asset": True,
                "parent_ordinals": True,
                "resumable_batch_transactions": True,
                "default_workers": int(args.workers),
                "timeout_seconds": float(args.timeout_seconds),
            },
            "prepared_at_utc": prepared_at,
        }
        manifest["manifest_content_sha256"] = self_hash(
            manifest, "manifest_content_sha256"
        )
        atomic_json(manifest_path, manifest)
        return manifest
    finally:
        if connection is not None:
            connection.close()
        audit_temporary.unlink(missing_ok=True)
        database_temporary.unlink(missing_ok=True)


def _require_current_sources(manifest: Mapping[str, Any]) -> None:
    expected = manifest.get("correction_sources")
    if not isinstance(expected, Mapping):
        raise CorrectionError("prepared manifest lacks correction source bindings")
    current = _current_source_bindings()
    for key, binding in current.items():
        if dict(expected.get(key, {})) != binding:
            raise CorrectionError(f"correction source drift after prepare: {key}")


def _load_prepared(
    output: Path, *, verify_databases: bool = False
) -> tuple[dict[str, Any], dict[str, Any]]:
    output = Path(output).resolve(strict=True)
    manifest = load_json(output / "manifest.json", "correction manifest")
    if manifest.get("schema_version") != RUN_SCHEMA:
        raise CorrectionError("correction manifest schema mismatch")
    require_self_hash(manifest, "manifest_content_sha256", "correction manifest")
    if Path(str(manifest.get("output", ""))).resolve(strict=True) != output:
        raise CorrectionError("correction output path binding mismatch")
    _require_current_sources(manifest)
    audit = verify_binding(manifest["selection"]["audit"], "selection audit")
    if audit != (output / AUDIT_NAME).resolve(strict=True):
        raise CorrectionError("selection audit output binding mismatch")
    parent = Path(str(manifest["parent"]["root"]))
    snapshot = Path(manifest["parent_source_snapshot"]["core"]["path"]).parent
    context = _parent_context(parent, snapshot, verify_databases=verify_databases)
    if context["receipt"]["receipt_content_sha256"] != manifest["parent"].get(
        "receipt_content_sha256"
    ):
        raise CorrectionError("prepared parent receipt binding mismatch")
    if _database_declared_binding(
        context["result_database"], context["result_database_sha256"]
    ) != dict(manifest["parent"]["result_database"]):
        raise CorrectionError("prepared parent database binding mismatch")
    if _database_declared_binding(
        context["source_database"], context["source_database_sha256"]
    ) != dict(manifest["source_roster"]["database"]):
        raise CorrectionError("prepared source database binding mismatch")
    return manifest, context


def _decode_states(blob: bytes, expected_count: int) -> list[dict[str, Any]]:
    try:
        raw = zlib.decompress(blob)
        lines = raw.splitlines()
        states = [json.loads(line) for line in lines]
    except (zlib.error, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CorrectionError(f"invalid compressed state payload: {error}") from error
    if len(states) != expected_count or any(not isinstance(row, dict) for row in states):
        raise CorrectionError("state payload count/type mismatch")
    return states


def _validate_overlay_payload(
    payload: Mapping[str, Any], selection: Mapping[str, Any]
) -> dict[str, Any]:
    ordinal = int(selection["parent_ordinal"])
    asset_id = str(selection["asset_id"])
    if int(payload.get("ordinal", -1)) != ordinal or str(
        payload.get("asset_id", "")
    ) != asset_id:
        raise CorrectionError(f"worker identity mismatch at parent ordinal {ordinal}")
    try:
        record = json.loads(str(payload["record_json"]))
    except (KeyError, json.JSONDecodeError) as error:
        raise CorrectionError(f"invalid worker record at ordinal {ordinal}") from error
    if not isinstance(record, dict):
        raise CorrectionError(f"non-object worker record at ordinal {ordinal}")
    states = _decode_states(bytes(payload["states_zlib"]), int(payload["state_count"]))
    if (
        int(record.get("order", -1)) != ordinal
        or str(record.get("dataset_id", "")) != asset_id
        or int(record.get("state_records_count", -1)) != len(states)
        or record.get("state_records_sha256") != canonical_sha256(states)
        or record.get("joint_sampling_plan_sha256")
        != selection["new_plan_sha256"]
        or int(record.get("independent_dof_count", -1))
        != int(selection["new_independent_dof"])
        or int(record.get("fixed_root_joint_count", -1))
        != int(selection["new_fixed_root_count"])
    ):
        raise CorrectionError(f"corrected result binding mismatch at ordinal {ordinal}")
    for state in states:
        if int(state.get("order", -1)) != ordinal or str(
            state.get("dataset_id", "")
        ) != asset_id:
            raise CorrectionError(f"corrected state identity mismatch at ordinal {ordinal}")
    return record


def _selection_row(row: Sequence[Any]) -> dict[str, Any]:
    keys = (
        "selection_index",
        "parent_ordinal",
        "asset_id",
        "source_row_sha256",
        "reasons_json",
        "old_execution_sig_sha256",
        "new_execution_sig_sha256",
        "old_scale_pose_sha256",
        "new_scale_pose_sha256",
        "new_plan_sha256",
        "new_independent_dof",
        "new_fixed_root_count",
        "audit_json",
        "row_content_sha256",
    )
    value = dict(zip(keys, row))
    value["reasons"] = json.loads(value.pop("reasons_json"))
    return value


SELECTION_COLUMNS = (
    "s.selection_index,s.parent_ordinal,s.asset_id,s.source_row_sha256,"
    "s.reasons_json,s.old_execution_sig_sha256,s.new_execution_sig_sha256,"
    "s.old_scale_pose_sha256,s.new_scale_pose_sha256,s.new_plan_sha256,"
    "s.new_independent_dof,s.new_fixed_root_count,s.audit_json,"
    "s.row_content_sha256"
)


def _write_progress(output: Path, connection: sqlite3.Connection, total: int) -> None:
    completed, states = connection.execute(
        "SELECT COUNT(*), COALESCE(SUM(state_count),0) FROM results"
    ).fetchone()
    worker_counts = dict(
        connection.execute(
            "SELECT worker_status,COUNT(*) FROM results GROUP BY worker_status "
            "ORDER BY worker_status"
        )
    )
    value: dict[str, Any] = {
        "schema_version": CHECKPOINT_SCHEMA,
        "status": "COMPLETE" if int(completed) == total else "RUNNING",
        "completed_assets": int(completed),
        "selected_assets": int(total),
        "overlay_state_records": int(states),
        "worker_status_counts": worker_counts,
        "updated_at_utc": utc_now(),
    }
    value["progress_content_sha256"] = self_hash(value, "progress_content_sha256")
    atomic_json(output / "progress.json", value)


def _run(args: argparse.Namespace) -> dict[str, Any]:
    output = Path(args.output).resolve(strict=True)
    manifest, context = _load_prepared(
        output, verify_databases=bool(args.rehash_input_databases)
    )
    database_path = output / DATABASE_NAME
    connection = connect_overlay(database_path)
    source = connect_immutable(
        context["source_database"], context["source_database_sha256"], verify_hash=False
    )
    try:
        meta = _decoded_meta(connection)
        if meta.get("schema_version") != OVERLAY_DB_SCHEMA:
            raise CorrectionError("overlay database schema mismatch")
        if meta.get("run_state") == "COMPLETE":
            raise CorrectionError("overlay is already finalized")
        selection_total = int(
            connection.execute("SELECT COUNT(*) FROM selection").fetchone()[0]
        )
        if selection_total != int(manifest["selection"]["selected_asset_count"]):
            raise CorrectionError("overlay selection count drift")
        pending = list(
            connection.execute(
                f"SELECT {SELECTION_COLUMNS} FROM selection s "
                "LEFT JOIN results r ON r.parent_ordinal=s.parent_ordinal "
                "WHERE r.parent_ordinal IS NULL ORDER BY s.selection_index"
            )
        )
        if args.max_assets is not None:
            pending = pending[: int(args.max_assets)]
        workers = int(args.workers)
        batch_size = int(args.batch_size)
        if workers <= 0 or batch_size <= 0 or float(args.timeout_seconds) <= 0:
            raise CorrectionError("workers, batch size, and timeout must be positive")
        with ThreadPoolExecutor(max_workers=workers) as executor:
            for start in range(0, len(pending), batch_size):
                _require_current_sources(manifest)
                selected = [_selection_row(row) for row in pending[start : start + batch_size]]
                work: list[tuple[dict[str, Any], dict[str, Any]]] = []
                for selection in selected:
                    source_value = source.execute(
                        "SELECT asset_id,row_sha256,row_json FROM assets WHERE ordinal=?",
                        (int(selection["parent_ordinal"]),),
                    ).fetchone()
                    if source_value is None:
                        raise CorrectionError("selected source row is missing")
                    asset_id, row_hash, row_json = source_value
                    if (
                        str(asset_id) != selection["asset_id"]
                        or str(row_hash) != selection["source_row_sha256"]
                    ):
                        raise CorrectionError("selected source row binding drift")
                    row = json.loads(row_json)
                    work.append((selection, row))
                futures = {
                    executor.submit(
                        pva_adapter._evaluate_row,
                        row,
                        output,
                        float(args.timeout_seconds),
                        context["package_binding"],
                    ): selection
                    for selection, row in work
                }
                completed: list[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]] = []
                for future in as_completed(futures):
                    selection = futures[future]
                    payload = future.result()
                    record = _validate_overlay_payload(payload, selection)
                    completed.append((selection, payload, record))
                _require_current_sources(manifest)
                completed.sort(key=lambda value: int(value[0]["selection_index"]))
                with connection:
                    for selection, payload, _record in completed:
                        connection.execute(
                            "INSERT INTO results(parent_ordinal,asset_id,record_json,"
                            "states_zlib,state_count,worker_status,worker_seconds,"
                            "completed_at_utc) VALUES(?,?,?,?,?,?,?,?)",
                            (
                                int(selection["parent_ordinal"]),
                                str(selection["asset_id"]),
                                str(payload["record_json"]),
                                sqlite3.Binary(bytes(payload["states_zlib"])),
                                int(payload["state_count"]),
                                str(payload["worker_status"]),
                                float(payload["worker_seconds"]),
                                str(payload["completed_at_utc"]),
                            ),
                        )
                for selection, _payload, record in completed:
                    if str(record.get("status")) == "completed":
                        ordinal = int(selection["parent_ordinal"])
                        for directory, suffix in (
                            ("child_jobs", ".json"),
                            ("children", ".json"),
                            ("child_logs", ".log"),
                        ):
                            (output / directory / f"{ordinal:06d}{suffix}").unlink(
                                missing_ok=True
                            )
                _write_progress(output, connection, selection_total)
        _write_progress(output, connection, selection_total)
        completed_count = int(
            connection.execute("SELECT COUNT(*) FROM results").fetchone()[0]
        )
        return {
            "selected_assets": selection_total,
            "completed_assets": completed_count,
            "pending_assets": selection_total - completed_count,
        }
    finally:
        source.close()
        connection.close()


class _EffectiveRecordStream:
    def __init__(
        self,
        parent_database: Path,
        parent_sha256: str,
        overlay_database: Path,
        overlay_sha256: str,
        count: int,
    ) -> None:
        self.parent_database = parent_database
        self.parent_sha256 = parent_sha256
        self.overlay_database = overlay_database
        self.overlay_sha256 = overlay_sha256
        self.count = count
        self.hasher = CanonicalListHasher()
        self.observed = 0
        self._iterations = 0
        self._hash_complete = False

    def __len__(self) -> int:
        return self.count

    def __iter__(self) -> Iterator[dict[str, Any]]:
        hash_this_pass = self._iterations == 0
        self._iterations += 1
        observed = 0
        parent = connect_immutable(
            self.parent_database, self.parent_sha256, verify_hash=False
        )
        overlay = connect_immutable(
            self.overlay_database, self.overlay_sha256, verify_hash=False
        )
        overlay_cursor = overlay.execute(
            "SELECT parent_ordinal,record_json FROM results ORDER BY parent_ordinal"
        )
        replacement = overlay_cursor.fetchone()
        try:
            for ordinal, parent_json in parent.execute(
                "SELECT ordinal,record_json FROM results ORDER BY ordinal"
            ):
                if int(ordinal) != observed:
                    raise CorrectionError("parent result ordinals are not contiguous")
                if replacement is not None and int(replacement[0]) == int(ordinal):
                    record_json = replacement[1]
                    replacement = overlay_cursor.fetchone()
                else:
                    record_json = parent_json
                record = json.loads(record_json)
                if not isinstance(record, dict):
                    raise CorrectionError("effective record is not an object")
                if hash_this_pass:
                    self.hasher.add(record)
                observed += 1
                yield record
            if replacement is not None or observed != self.count:
                raise CorrectionError("effective record closure mismatch")
            if hash_this_pass:
                self.observed = observed
                self._hash_complete = True
        finally:
            parent.close()
            overlay.close()

    def digest(self) -> str:
        if not self._hash_complete:
            raise CorrectionError("effective record digest is incomplete")
        digest, count = self.hasher.finish()
        if count != self.count or self.observed != self.count:
            raise CorrectionError("effective record digest is incomplete")
        return digest


def _artifact_manifest(output: Path, paths: Sequence[Path]) -> dict[str, Any]:
    artifacts = [
        {
            "path": path.relative_to(output).as_posix(),
            "bytes": int(path.stat().st_size),
            "sha256": sha256_file(path),
        }
        for path in sorted(paths)
    ]
    value: dict[str, Any] = {
        "schema_version": ARTIFACT_SCHEMA,
        "artifacts": artifacts,
    }
    value["artifact_manifest_content_sha256"] = self_hash(
        value, "artifact_manifest_content_sha256"
    )
    return value


def _finalize(args: argparse.Namespace) -> dict[str, Any]:
    output = Path(args.output).resolve(strict=True)
    existing_receipt = output / "full_release_receipt.json"
    if existing_receipt.exists():
        receipt = load_json(existing_receipt, "correction receipt")
        require_self_hash(receipt, "receipt_content_sha256", "correction receipt")
        return load_json(output / "summary.json", "correction summary")
    manifest, context = _load_prepared(
        output, verify_databases=bool(args.rehash_input_databases)
    )
    database_path = output / DATABASE_NAME
    connection = connect_overlay(database_path)
    try:
        selected = int(connection.execute("SELECT COUNT(*) FROM selection").fetchone()[0])
        completed = int(connection.execute("SELECT COUNT(*) FROM results").fetchone()[0])
        if selected != int(manifest["selection"]["selected_asset_count"]):
            raise CorrectionError("selection count changed before finalize")
        if completed != selected:
            raise CorrectionError(
                f"overlay is incomplete: {completed}/{selected} selected assets"
            )
        with connection:
            _set_meta(
                connection,
                {
                    "run_state": "COMPLETE",
                    "completed_asset_count": completed,
                    "finalized_at_utc": utc_now(),
                },
            )
        if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
            raise CorrectionError("overlay database integrity check failed")
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        connection.execute("PRAGMA journal_mode=DELETE")
    finally:
        connection.close()
    overlay_sha256 = sha256_file(database_path)
    n_eval = int(manifest["N_eval"])
    stream = _EffectiveRecordStream(
        context["result_database"],
        context["result_database_sha256"],
        database_path,
        overlay_sha256,
        n_eval,
    )
    summary = table4.aggregate_records(stream, n_eval, int(manifest["J_eval"]))
    records_sha256 = stream.digest()
    totals = manifest["selection"]["cohort_totals"]
    summary.update(
        {
            "schema_version": SUMMARY_SCHEMA,
            "dataset": "Ours / PV-A",
            "dataset_slug": "pva",
            "effective_protocol_id": EFFECTIVE_PROTOCOL_ID,
            "sampling_protocol": table4.SAMPLING_PROTOCOL_V2,
            "overlay_policy": OVERLAY_POLICY,
            "declared_dof_count": int(manifest["J_eval"]),
            "independent_dof_count": int(totals["new_independent_dof"]),
            "range_evaluable_independent_dof_count": int(
                totals["new_range_evaluable_independent_dof"]
            ),
            "mimic_joint_count": int(totals["mimic_joint_count"]),
            "fixed_root_joint_count": int(totals["fixed_root_joint_count"]),
            "selected_correction_assets": selected,
            "parent_receipt_content_sha256": manifest["parent"][
                "receipt_content_sha256"
            ],
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "overlay_database_sha256": overlay_sha256,
            "effective_records_sha256": records_sha256,
            "states_materialization": "sparse_overlay_only",
            "completed_at_utc": utc_now(),
        }
    )
    summary["summary_content_sha256"] = self_hash(
        summary, "summary_content_sha256"
    )
    atomic_json(output / "summary.json", summary)
    progress: dict[str, Any] = {
        "schema_version": CHECKPOINT_SCHEMA,
        "status": "COMPLETE",
        "completed_assets": selected,
        "selected_assets": selected,
        "effective_assets": n_eval,
        "updated_at_utc": utc_now(),
    }
    progress["progress_content_sha256"] = self_hash(
        progress, "progress_content_sha256"
    )
    atomic_json(output / "progress.json", progress)
    checkpoint: dict[str, Any] = {
        "schema_version": CHECKPOINT_SCHEMA,
        "state": "complete",
        "selected_records": selected,
        "effective_records": n_eval,
        "effective_records_sha256": records_sha256,
        "overlay_database_sha256": overlay_sha256,
        "summary_sha256": sha256_file(output / "summary.json"),
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed_at_utc": utc_now(),
    }
    checkpoint["checkpoint_content_sha256"] = self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    atomic_json(output / "checkpoint.json", checkpoint)
    artifact_paths = [
        output / "manifest.json",
        output / AUDIT_NAME,
        database_path,
        output / "summary.json",
        output / "checkpoint.json",
        output / "progress.json",
    ]
    for binding in manifest["parent_source_snapshot"].values():
        path = Path(binding["path"]).resolve(strict=True)
        try:
            path.relative_to(output)
        except ValueError:
            continue
        artifact_paths.append(path)
    artifact = _artifact_manifest(output, artifact_paths)
    atomic_json(output / "artifact_manifest.json", artifact)
    receipt: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA,
        "classification": "FORMAL_FULL_RELEASE_SPARSE_CORRECTION",
        "dataset": "Ours / PV-A",
        "dataset_slug": "pva",
        "effective_protocol_id": EFFECTIVE_PROTOCOL_ID,
        "overlay_policy": OVERLAY_POLICY,
        "N_eval": n_eval,
        "J_eval": int(manifest["J_eval"]),
        "selected_correction_assets": selected,
        "parent": manifest["parent"],
        "manifest": "manifest.json",
        "manifest_sha256": sha256_file(output / "manifest.json"),
        "selection_audit": AUDIT_NAME,
        "selection_audit_sha256": sha256_file(output / AUDIT_NAME),
        "overlay_database": DATABASE_NAME,
        "overlay_database_sha256": overlay_sha256,
        "effective_records_sha256": records_sha256,
        "summary": "summary.json",
        "summary_sha256": sha256_file(output / "summary.json"),
        "artifact_manifest": "artifact_manifest.json",
        "artifact_manifest_sha256": sha256_file(output / "artifact_manifest.json"),
        "status": summary["status"],
        "metrics": summary["metrics"],
        "completed_at_utc": utc_now(),
    }
    receipt["receipt_content_sha256"] = self_hash(
        receipt, "receipt_content_sha256"
    )
    atomic_json(existing_receipt, receipt)
    return summary


def _state_key(state: Mapping[str, Any]) -> tuple[Any, ...]:
    return (
        state.get("phase"),
        state.get("joint_name"),
        int(state.get("sample_index", -1)),
        state.get("joint_values_sha256"),
    )


def _state_map(states: Sequence[Mapping[str, Any]], label: str) -> dict[tuple[Any, ...], Mapping[str, Any]]:
    result: dict[tuple[Any, ...], Mapping[str, Any]] = {}
    for state in states:
        key = _state_key(state)
        if key in result:
            raise CorrectionError(f"duplicate semantic state in {label}: {key[:3]}")
        result[key] = state
    return result


def _require_raw_state_equal(
    old: Mapping[str, Any], new: Mapping[str, Any], ordinal: int
) -> None:
    for field in RAW_STATE_FIELDS:
        if old.get(field) != new.get(field):
            raise CorrectionError(
                f"raw state parity mismatch at ordinal {ordinal}: {field}"
            )


def _verify_state_parity(
    selection: Mapping[str, Any], old_states: list[dict[str, Any]], new_states: list[dict[str, Any]]
) -> dict[str, int]:
    ordinal = int(selection["parent_ordinal"])
    reasons = set(selection["reasons"])
    old_by_key = _state_map(old_states, f"parent ordinal {ordinal}")
    new_by_key = _state_map(new_states, f"overlay ordinal {ordinal}")
    old_keys = set(old_by_key)
    new_keys = set(new_by_key)
    common = old_keys & new_keys
    for key in common:
        _require_raw_state_equal(old_by_key[key], new_by_key[key], ordinal)
    old_only = old_keys - new_keys
    new_only = new_keys - old_keys
    if reasons == {"expanded_rest_normalizer"}:
        if old_only or new_only:
            raise CorrectionError(
                f"scale-only correction changed state identities at ordinal {ordinal}"
            )
    elif "zero_width_fixed_root" in reasons:
        invalid_new = [key for key in new_only if key[0] != "multi_joint_sobol"]
        audit = json.loads(str(selection["audit_json"]))
        removed_roots = int(audit["old_independent_dof"]) - int(
            audit["new_independent_dof"]
        )
        if (
            old_only
            or invalid_new
            or removed_roots <= 0
            or len(new_only)
            not in (
                0,
                int(table4.SOBOL_SAMPLES),
            )
        ):
            raise CorrectionError(
                f"zero-width state delta is not protocol-shaped at ordinal {ordinal}"
            )
    elif reasons == {"backend_capacity_retry"}:
        if old_only:
            raise CorrectionError(
                f"backend retry removed parent states at ordinal {ordinal}"
            )
    else:
        raise CorrectionError(f"unknown parity rule at ordinal {ordinal}: {sorted(reasons)}")
    return {
        "common": len(common),
        "parent_only": len(old_only),
        "overlay_only": len(new_only),
    }


def _verify_selection_rows(
    connection: sqlite3.Connection, manifest: Mapping[str, Any]
) -> dict[str, Any]:
    hasher = CanonicalListHasher()
    count = 0
    for row in connection.execute(
        f"SELECT {SELECTION_COLUMNS} FROM selection s ORDER BY s.selection_index"
    ):
        selection = _selection_row(row)
        if int(selection["selection_index"]) != count:
            raise CorrectionError(f"selection index is not contiguous at {count}")
        audit = json.loads(str(selection["audit_json"]))
        if not isinstance(audit, dict) or not audit.get("selected"):
            raise CorrectionError(f"invalid selection audit row at {count}")
        require_self_hash(audit, "row_content_sha256", f"selection row {count}")
        checks = {
            "parent_ordinal": int(selection["parent_ordinal"]),
            "asset_id": str(selection["asset_id"]),
            "source_row_sha256": str(selection["source_row_sha256"]),
            "selection_reasons": selection["reasons"],
            "old_execution_sig_sha256": selection["old_execution_sig_sha256"],
            "new_execution_sig_sha256": selection["new_execution_sig_sha256"],
            "old_scale_pose_sha256": selection["old_scale_pose_sha256"],
            "new_scale_pose_sha256": selection["new_scale_pose_sha256"],
            "new_plan_sha256": selection["new_plan_sha256"],
            "new_independent_dof": int(selection["new_independent_dof"]),
            "new_fixed_root_count": int(selection["new_fixed_root_count"]),
            "row_content_sha256": selection["row_content_sha256"],
        }
        for key, value in checks.items():
            if audit.get(key) != value:
                raise CorrectionError(f"selection column/audit mismatch at {count}: {key}")
        hasher.add(
            {
                "parent_ordinal": int(selection["parent_ordinal"]),
                "asset_id": str(selection["asset_id"]),
                "source_row_sha256": str(selection["source_row_sha256"]),
                "selection_reasons": selection["reasons"],
            }
        )
        count += 1
    digest, hashed_count = hasher.finish()
    if (
        count != hashed_count
        or count != int(manifest["selection"]["selected_asset_count"])
        or digest != manifest["selection"]["selected_assets_sha256"]
    ):
        raise CorrectionError("selection table identity closure mismatch")
    return {"selected_asset_count": count, "selected_assets_sha256": digest}


def _verify_artifacts(output: Path, receipt: Mapping[str, Any]) -> dict[str, Any]:
    artifact_path = output / str(receipt["artifact_manifest"])
    if sha256_file(artifact_path) != receipt.get("artifact_manifest_sha256"):
        raise CorrectionError("correction artifact manifest receipt binding mismatch")
    artifact = load_json(artifact_path, "correction artifact manifest")
    if artifact.get("schema_version") != ARTIFACT_SCHEMA:
        raise CorrectionError("correction artifact manifest schema mismatch")
    require_self_hash(
        artifact,
        "artifact_manifest_content_sha256",
        "correction artifact manifest",
    )
    seen: set[str] = set()
    for row in artifact.get("artifacts", []):
        if not isinstance(row, Mapping):
            raise CorrectionError("artifact manifest contains a non-object row")
        relative = Path(str(row.get("path", "")))
        if relative.is_absolute() or ".." in relative.parts or relative.as_posix() in seen:
            raise CorrectionError("artifact manifest contains an unsafe/duplicate path")
        seen.add(relative.as_posix())
        path = (output / relative).resolve(strict=True)
        path.relative_to(output)
        binding = file_binding(path, f"artifact {relative}")
        if binding["bytes"] != row.get("bytes") or binding["sha256"] != row.get(
            "sha256"
        ):
            raise CorrectionError(f"artifact binding mismatch: {relative}")
    required = {
        "manifest.json",
        AUDIT_NAME,
        DATABASE_NAME,
        "summary.json",
        "checkpoint.json",
        "progress.json",
    }
    if not required.issubset(seen):
        raise CorrectionError("artifact manifest lacks required correction artifacts")
    return artifact


def _verify(args: argparse.Namespace) -> dict[str, Any]:
    output = Path(args.output).resolve(strict=True)
    receipt_path = output / "full_release_receipt.json"
    receipt = load_json(receipt_path, "correction receipt")
    if receipt.get("schema_version") != RECEIPT_SCHEMA:
        raise CorrectionError("correction receipt schema mismatch")
    require_self_hash(receipt, "receipt_content_sha256", "correction receipt")
    artifact = _verify_artifacts(output, receipt)
    manifest, context = _load_prepared(output, verify_databases=True)
    for filename, field in (
        ("manifest.json", "manifest_sha256"),
        (AUDIT_NAME, "selection_audit_sha256"),
        ("summary.json", "summary_sha256"),
    ):
        if sha256_file(output / filename) != receipt.get(field):
            raise CorrectionError(f"receipt file binding mismatch: {filename}")
    database_path = output / DATABASE_NAME
    overlay_sha256 = sha256_file(database_path)
    if overlay_sha256 != receipt.get("overlay_database_sha256"):
        raise CorrectionError("receipt overlay database binding mismatch")
    connection = connect_immutable(database_path, overlay_sha256, verify_hash=False)
    try:
        if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
            raise CorrectionError("overlay database integrity check failed")
        meta = _decoded_meta(connection)
        if meta.get("schema_version") != OVERLAY_DB_SCHEMA or meta.get("run_state") != "COMPLETE":
            raise CorrectionError("overlay database is not finalized")
        selection_closure = _verify_selection_rows(connection, manifest)
        selected_rows = [
            _selection_row(row)
            for row in connection.execute(
                f"SELECT {SELECTION_COLUMNS} FROM selection s ORDER BY s.selection_index"
            )
        ]
        result_count = int(connection.execute("SELECT COUNT(*) FROM results").fetchone()[0])
        if result_count != len(selected_rows):
            raise CorrectionError("overlay result/selection count mismatch")
        old_core, new_core = _load_cores(context)
        with (output / AUDIT_NAME).open("rb") as expected_audit:
            selected_cursor = iter(selected_rows)

            def compare_selected(index: int, detail: Mapping[str, Any]) -> None:
                try:
                    stored = next(selected_cursor)
                except StopIteration as error:
                    raise CorrectionError("selection rescan produced extra rows") from error
                if int(stored["selection_index"]) != index or stored[
                    "audit_json"
                ] != canonical_text(detail):
                    raise CorrectionError(f"selection rescan drift at index {index}")

            scan = _scan_selection(
                context,
                old_core,
                new_core,
                include_backend_capacity_errors=bool(
                    manifest["selection"]["include_backend_capacity_errors"]
                ),
                expected_audit_stream=expected_audit,
                selected_callback=compare_selected,
            )
            try:
                next(selected_cursor)
            except StopIteration:
                pass
            else:
                raise CorrectionError("selection rescan omitted stored rows")
        for key, value in scan.items():
            if manifest["selection"].get(key) != value:
                raise CorrectionError(f"selection scan manifest drift: {key}")
        parent = connect_immutable(
            context["result_database"],
            context["result_database_sha256"],
            verify_hash=False,
        )
        parity_counts: Counter[str] = Counter()
        try:
            for selection in selected_rows:
                ordinal = int(selection["parent_ordinal"])
                parent_row = parent.execute(
                    "SELECT asset_id,record_json,states_zlib,state_count FROM results "
                    "WHERE ordinal=?",
                    (ordinal,),
                ).fetchone()
                overlay_row = connection.execute(
                    "SELECT asset_id,record_json,states_zlib,state_count,worker_status,"
                    "worker_seconds,completed_at_utc FROM results WHERE parent_ordinal=?",
                    (ordinal,),
                ).fetchone()
                if parent_row is None or overlay_row is None:
                    raise CorrectionError(f"missing parity row at ordinal {ordinal}")
                parent_id, parent_json, parent_blob, parent_count = parent_row
                (
                    overlay_id,
                    overlay_json,
                    overlay_blob,
                    overlay_count,
                    worker_status,
                    worker_seconds,
                    completed_at,
                ) = overlay_row
                if str(parent_id) != selection["asset_id"] or str(overlay_id) != selection[
                    "asset_id"
                ]:
                    raise CorrectionError(f"parity asset mismatch at ordinal {ordinal}")
                parent_record = json.loads(parent_json)
                parent_states = _decode_states(bytes(parent_blob), int(parent_count))
                payload = {
                    "ordinal": ordinal,
                    "asset_id": overlay_id,
                    "record_json": overlay_json,
                    "states_zlib": overlay_blob,
                    "state_count": overlay_count,
                    "worker_status": worker_status,
                    "worker_seconds": worker_seconds,
                    "completed_at_utc": completed_at,
                }
                overlay_record = _validate_overlay_payload(payload, selection)
                overlay_states = _decode_states(bytes(overlay_blob), int(overlay_count))
                try:
                    old_core.validate_state_closure(parent_record, parent_states)
                    new_core.validate_state_closure(overlay_record, overlay_states)
                except BaseException as error:
                    raise CorrectionError(
                        f"state closure failed at ordinal {ordinal}: {error}"
                    ) from error
                if str(overlay_record.get("status")) == "completed":
                    if not overlay_states:
                        raise CorrectionError(
                            f"completed overlay has no states at ordinal {ordinal}"
                        )
                    scale = float(overlay_record["object_bbox_diagonal_m"])
                    maximum = max(
                        float(state["metric_max_penetration_m"])
                        for state in overlay_states
                    )
                    normalized = float(
                        overlay_record["max_penetration_normalized"]
                    )
                    if (
                        not math.isfinite(scale)
                        or scale <= 0.0
                        or not math.isfinite(maximum)
                        or maximum < 0.0
                        or float(overlay_record["max_penetration_m"]) != maximum
                        or not math.isclose(
                            normalized,
                            maximum / scale,
                            rel_tol=1e-12,
                            abs_tol=1e-15,
                        )
                    ):
                        raise CorrectionError(
                            f"overlay normalization arithmetic mismatch at {ordinal}"
                        )
                    if (
                        overlay_record.get("normalization_configuration")
                        != "expanded_rest"
                        or overlay_record.get("normalization_joint_values_sha256")
                        != selection["new_scale_pose_sha256"]
                    ):
                        raise CorrectionError(
                            f"overlay expanded-rest receipt mismatch at {ordinal}"
                        )
                counts = _verify_state_parity(
                    selection, parent_states, overlay_states
                )
                parity_counts.update(counts)
                reasons = set(selection["reasons"])
                if "expanded_rest_normalizer" in reasons and str(
                    overlay_record.get("status")
                ) == "completed":
                    if overlay_record.get("max_penetration_m") != parent_record.get(
                        "max_penetration_m"
                    ):
                        raise CorrectionError(
                            f"expanded-rest normalization binding mismatch at {ordinal}"
                        )
        finally:
            parent.close()
    finally:
        connection.close()
    summary = load_json(output / "summary.json", "correction summary")
    require_self_hash(summary, "summary_content_sha256", "correction summary")
    stream = _EffectiveRecordStream(
        context["result_database"],
        context["result_database_sha256"],
        database_path,
        overlay_sha256,
        int(manifest["N_eval"]),
    )
    recomputed = table4.aggregate_records(
        stream, int(manifest["N_eval"]), int(manifest["J_eval"])
    )
    effective_hash = stream.digest()
    for key, value in recomputed.items():
        if key != "schema_version" and summary.get(key) != value:
            raise CorrectionError(f"effective aggregate mismatch: {key}")
    if (
        effective_hash != summary.get("effective_records_sha256")
        or effective_hash != receipt.get("effective_records_sha256")
        or summary.get("metrics") != receipt.get("metrics")
        or summary.get("summary_content_sha256")
        != self_hash(summary, "summary_content_sha256")
    ):
        raise CorrectionError("effective summary/receipt closure mismatch")
    verification: dict[str, Any] = {
        "schema_version": VERIFICATION_SCHEMA,
        "status": "PASS",
        "verified_at_utc": utc_now(),
        "receipt_sha256": sha256_file(receipt_path),
        "parent_receipt_content_sha256": context["receipt"][
            "receipt_content_sha256"
        ],
        "parent_result_database_sha256": context["result_database_sha256"],
        "source_result_database_sha256": context["source_database_sha256"],
        "overlay_database_sha256": overlay_sha256,
        "selection": selection_closure,
        "selection_scan": scan,
        "selected_state_parity_counts": dict(sorted(parity_counts.items())),
        "effective_record_count": int(manifest["N_eval"]),
        "effective_records_sha256": effective_hash,
        "effective_aggregate_recomputed": True,
        "full_parent_and_source_database_hashes_recomputed": True,
        "artifact_count": len(artifact["artifacts"]),
    }
    verification["verification_content_sha256"] = self_hash(
        verification, "verification_content_sha256"
    )
    atomic_json(output / "verification.json", verification)
    return verification


def _fixture_record(ordinal: int, *, passed: bool) -> dict[str, Any]:
    return {
        "order": ordinal,
        "dataset_id": f"fixture-{ordinal}",
        "status": "completed",
        "movable_dof_count": 1,
        "native_collision_elements": 1,
        "collision_metric_status": "measured",
        "measurement_complete": True,
        "rest_state_expected": 1,
        "rest_state_executed": 1,
        "rest_non_adjacent_free": int(passed),
        "single_state_expected": 21,
        "single_state_executed": 21,
        "single_non_adjacent_free": 21 * int(passed),
        "sobol_state_expected": 64,
        "sobol_state_executed": 64,
        "sobol_non_adjacent_free": 64 * int(passed),
        "rest_all_pair_cf": passed,
        "rest_non_adjacent_cf": passed,
        "single_joint_sweep_cf": passed,
        "multi_joint_sobol_cf": passed,
        "strict_collision_pass": passed,
        "max_penetration_normalized": 0.0 if passed else 0.1,
    }


def _self_test() -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="pva-v2-sparse-fixture-") as name:
        root = Path(name)
        parent_path = root / "parent.sqlite3"
        overlay_path = root / "overlay.sqlite3"
        parent = sqlite3.connect(parent_path)
        parent.execute(
            "CREATE TABLE results(ordinal INTEGER PRIMARY KEY,record_json TEXT NOT NULL)"
        )
        parent.executemany(
            "INSERT INTO results VALUES(?,?)",
            [
                (0, canonical_text(_fixture_record(0, passed=True))),
                (1, canonical_text(_fixture_record(1, passed=True))),
            ],
        )
        parent.commit()
        parent.close()
        overlay = connect_overlay(overlay_path)
        _create_overlay_schema(overlay)
        for index in range(1):
            detail = {
                "parent_ordinal": 1,
                "asset_id": "fixture-1",
                "source_row_sha256": "fixture-source",
                "selection_reasons": ["expanded_rest_normalizer"],
                "old_execution_sig_sha256": "old",
                "new_execution_sig_sha256": "new",
                "old_scale_pose_sha256": canonical_sha256([0.0]),
                "new_scale_pose_sha256": canonical_sha256([0.0]),
                "new_plan_sha256": "plan",
                "new_independent_dof": 1,
                "new_fixed_root_count": 0,
                "selected": True,
                "row_content_sha256": "placeholder",
            }
            detail["row_content_sha256"] = self_hash(
                detail, "row_content_sha256"
            )
            _selection_insert(overlay, 0, {
                **detail,
                "selection_reasons": detail["selection_reasons"],
            })
        replacement = _fixture_record(1, passed=False)
        overlay.execute(
            "INSERT INTO results(parent_ordinal,asset_id,record_json,states_zlib,"
            "state_count,worker_status,worker_seconds,completed_at_utc) "
            "VALUES(?,?,?,?,?,?,?,?)",
            (
                1,
                "fixture-1",
                canonical_text(replacement),
                sqlite3.Binary(zlib.compress(b"")),
                0,
                "completed",
                0.0,
                utc_now(),
            ),
        )
        overlay.commit()
        overlay.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        overlay.execute("PRAGMA journal_mode=DELETE")
        overlay.close()
        parent_hash = sha256_file(parent_path)
        overlay_hash = sha256_file(overlay_path)
        stream = _EffectiveRecordStream(
            parent_path, parent_hash, overlay_path, overlay_hash, 2
        )
        summary = table4.aggregate_records(stream, 2, 2)
        digest = stream.digest()
        if (
            summary["metrics"]["strict_collision_pass"]["passed"] != 1
            or summary["status"] != "COMPLETE"
            or len(digest) != 64
        ):
            raise CorrectionError("effective-stream fixture failed")
        value = {"schema_version": "pva_v2_sparse_self_test_v1", "status": "PASS"}
        value["self_test_content_sha256"] = self_hash(
            value, "self_test_content_sha256"
        )
        require_self_hash(value, "self_test_content_sha256", "self-test")
        return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare = subparsers.add_parser("prepare", help="scan and freeze the sparse selector")
    prepare.add_argument("--parent", type=Path, default=DEFAULT_PARENT)
    prepare.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    prepare.add_argument(
        "--parent-source-snapshot", type=Path, default=DEFAULT_PARENT_SOURCE_SNAPSHOT
    )
    prepare.add_argument("--workers", type=int, default=32)
    prepare.add_argument("--timeout-seconds", type=float, default=600.0)
    prepare.add_argument("--batch-size", type=int, default=256)
    prepare.add_argument("--expected-correction-assets", type=int, default=847)
    prepare.add_argument("--include-backend-capacity-errors", action="store_true")
    prepare.add_argument("--high-capacity-runtime-attested", action="store_true")
    prepare.add_argument("--skip-input-database-rehash", action="store_true")

    run = subparsers.add_parser("run", help="evaluate pending selected parent ordinals")
    run.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    run.add_argument("--workers", type=int, default=32)
    run.add_argument("--timeout-seconds", type=float, default=600.0)
    run.add_argument("--batch-size", type=int, default=64)
    run.add_argument("--max-assets", type=int)
    run.add_argument("--rehash-input-databases", action="store_true")

    finalize = subparsers.add_parser(
        "finalize", help="stream the effective parent-plus-overlay aggregate"
    )
    finalize.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    finalize.add_argument("--rehash-input-databases", action="store_true")

    verify = subparsers.add_parser(
        "verify", help="fully verify bindings, selection, parity, and aggregate"
    )
    verify.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)

    subparsers.add_parser("self-test", help="run a lightweight synthetic fixture")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "prepare":
            result = _prepare(args)
            visible = {
                "status": "PREPARED",
                "output": result["output"],
                "scanned_asset_count": result["selection"]["scanned_asset_count"],
                "selected_asset_count": result["selection"]["selected_asset_count"],
                "reason_counts": result["selection"]["reason_counts"],
                "recognized_reason_counts": result["selection"][
                    "recognized_reason_counts"
                ],
            }
        elif args.command == "run":
            if args.max_assets is not None and args.max_assets < 0:
                raise CorrectionError("--max-assets cannot be negative")
            visible = _run(args)
        elif args.command == "finalize":
            summary = _finalize(args)
            visible = {
                "status": summary["status"],
                "n_eval": summary["n_eval"],
                "selected_correction_assets": summary[
                    "selected_correction_assets"
                ],
                "metrics": summary["metrics"],
            }
        elif args.command == "verify":
            visible = _verify(args)
        else:
            visible = _self_test()
    except (CorrectionError, OSError, sqlite3.Error, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 2
    print(json.dumps(visible, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
