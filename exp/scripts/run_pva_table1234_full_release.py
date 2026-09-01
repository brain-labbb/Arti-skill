#!/usr/bin/env python3
"""Run Table 1/2/2-supplementary/3/4 over the complete PV-A roster."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable, Iterator, Mapping, Sequence
import zlib


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import build_pva_full_release_roster as pva_roster
import run_table1_full_release as table1
import run_table2_full_release as table2
import run_table2sup_full_release as table2sup
import run_table3_full_release as table3
import run_table4_full_release as table4
import table123_full_release_common as common
import table2_supplementary_static as table2sup_static


RUN_SCHEMA_VERSION = "pva_table1234_full_release_run_v1"
RESULT_DB_SCHEMA_VERSION = "pva_table1234_results_db_v1"
RECEIPT_SCHEMA_VERSION = "pva_table1234_full_release_receipt_v1"
TABLE_NAMES = ("table1", "table2", "table2_supplementary", "table3", "table4")
RESULT_COLUMNS = {
    "table1": "table1_json",
    "table2": "table2_json",
    "table2_supplementary": "table2sup_json",
    "table3": "table3_json",
    "table4": "table4_json",
}
DEFAULT_ROSTER = (
    SCRIPT.parents[1]
    / "runtime"
    / "pva_table1234_full_release_20260826"
    / "roster"
    / "roster_manifest.json"
)
DEFAULT_OUTPUT = SCRIPT.parents[1] / "runtime" / "pva_table1234_full_release_20260826" / "evaluation"
PROTOCOL_DOCUMENT = SCRIPT.parents[1] / "URDF-Sim-Ready-Automatic-Evaluation.md"
BLAS_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def _canonical_text(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    return common.canonical_sha256(_without(value, field))


def _atomic_json(path: Path, value: Any) -> None:
    common._atomic_write_bytes(path, (_canonical_text(value) + "\n").encode("utf-8"))


def _atomic_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> int:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    count = 0
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            for row in rows:
                stream.write(_canonical_text(dict(row)) + "\n")
                count += 1
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
    return count


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _freeze_protocol(output: Path) -> dict[str, str]:
    source = PROTOCOL_DOCUMENT.resolve(strict=True)
    snapshot = output / "protocol_snapshot.md"
    if snapshot.exists():
        if not snapshot.is_file() or snapshot.is_symlink():
            raise ValueError(f"protocol snapshot is not a regular file: {snapshot}")
    else:
        common._atomic_write_bytes(snapshot, source.read_bytes())
    return {
        "source_document": str(source),
        "snapshot": snapshot.relative_to(output).as_posix(),
        "snapshot_sha256": common.sha256_file(snapshot),
    }


def _source_hashes() -> dict[str, str]:
    return {
        "runner": common.sha256_file(SCRIPT),
        "pva_roster_builder": common.sha256_file(Path(pva_roster.__file__)),
        "table123_common": common.sha256_file(Path(common.__file__)),
        "table1": common.sha256_file(Path(table1.__file__)),
        "table1_core": common.sha256_file(Path(table1.CORE_PATH)),
        "table2": common.sha256_file(Path(table2.__file__)),
        "table2_core": common.sha256_file(Path(table2.CORE_PATH)),
        "table2_supplementary": common.sha256_file(Path(table2sup.__file__)),
        "table2_supplementary_core": common.sha256_file(
            Path(table2sup_static.__file__)
        ),
        "table3": common.sha256_file(Path(table3.__file__)),
        "table3_core": common.sha256_file(Path(table3.CORE_PATH)),
        "table4": common.sha256_file(Path(table4.__file__)),
        "table4_core": common.sha256_file(Path(table4.CORE_SCRIPT)),
    }


def _connect(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(path, timeout=120.0)
    connection.execute("PRAGMA foreign_keys=ON")
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute("PRAGMA temp_store=MEMORY")
    return connection


def _create_schema(connection: sqlite3.Connection) -> None:
    connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS meta (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS assets (
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL CHECK (joint_count >= 0),
            row_sha256 TEXT NOT NULL,
            row_json TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS results (
            ordinal INTEGER PRIMARY KEY REFERENCES assets(ordinal),
            asset_id TEXT NOT NULL UNIQUE,
            table1_json TEXT NOT NULL,
            table2_json TEXT NOT NULL,
            table2sup_json TEXT NOT NULL,
            table3_json TEXT NOT NULL,
            table4_json TEXT NOT NULL,
            table4_states_zlib BLOB NOT NULL,
            table4_state_count INTEGER NOT NULL CHECK (table4_state_count >= 0),
            worker_status TEXT NOT NULL,
            worker_seconds REAL NOT NULL,
            completed_at_utc TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS results_asset_id ON results(asset_id);
        """
    )
    connection.commit()


def _meta(connection: sqlite3.Connection) -> dict[str, str]:
    return dict(connection.execute("SELECT key, value FROM meta"))


def _set_meta(connection: sqlite3.Connection, values: Mapping[str, Any]) -> None:
    connection.executemany(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        [(str(key), _canonical_text(value)) for key, value in values.items()],
    )


def _decode_meta(value: str) -> Any:
    return json.loads(value)


def _asset_import_binding(
    roster_manifest_path: Path,
    manifest: Mapping[str, Any],
    *,
    selected_n: int,
    limit: int | None,
) -> dict[str, Any]:
    return {
        "schema_version": RESULT_DB_SCHEMA_VERSION,
        "roster_manifest_sha256": common.sha256_file(roster_manifest_path),
        "roster_manifest_content_sha256": manifest["manifest_content_sha256"],
        "selected_n": selected_n,
        "limit": limit,
    }


def _initialize_assets(
    connection: sqlite3.Connection,
    roster_manifest_path: Path,
    *,
    limit: int | None,
) -> dict[str, Any]:
    manifest = pva_roster.load_roster_manifest(roster_manifest_path, verify_rows=False)
    declared_n = int(manifest["N_eval"])
    if limit is not None and (limit <= 0 or limit > declared_n):
        raise ValueError(f"limit must be in [1, {declared_n}]")
    selected_n = limit if limit is not None else declared_n
    binding = _asset_import_binding(
        roster_manifest_path,
        manifest,
        selected_n=selected_n,
        limit=limit,
    )
    existing_assets = int(connection.execute("SELECT COUNT(*) FROM assets").fetchone()[0])
    stored = _meta(connection)
    if stored:
        for key, value in binding.items():
            if key not in stored or _decode_meta(stored[key]) != value:
                raise ValueError(f"result database binding mismatch: {key}")
    elif existing_assets:
        raise ValueError("partial result database has no source binding")
    else:
        _set_meta(connection, {**binding, "asset_import_state": "LOADING"})
        connection.commit()

    import_state = (
        str(_decode_meta(stored["asset_import_state"]))
        if "asset_import_state" in stored
        else None
    )
    if import_state == "COMPLETE" and existing_assets != selected_n:
        raise ValueError("completed result database asset count mismatch")
    if existing_assets > selected_n:
        raise ValueError("result database has more assets than the selected roster")
    if existing_assets == selected_n and import_state in {None, "COMPLETE"}:
        j_eval = int(connection.execute("SELECT COALESCE(SUM(joint_count), 0) FROM assets").fetchone()[0])
        category_count = int(connection.execute("SELECT COUNT(DISTINCT category) FROM assets").fetchone()[0])
        return {
            "manifest": manifest,
            "n_eval": selected_n,
            "j_eval": j_eval,
            "category_count": category_count,
        }

    inserted = 0
    j_eval = 0
    categories: set[str] = set()
    batch: list[tuple[Any, ...]] = []
    existing_rows = connection.execute(
        "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
        "FROM assets ORDER BY ordinal"
    )
    for row in pva_roster.iter_roster_rows(roster_manifest_path):
        if inserted >= selected_n:
            break
        ordinal = int(row.get("ordinal", -1))
        if ordinal != inserted:
            raise ValueError(f"PV-A selected roster ordinal mismatch: {ordinal} != {inserted}")
        asset_id = str(row.get("asset_id", ""))
        category = str(row.get("raw_category", row.get("category", "")))
        joints = int(row.get("joint_count", -1))
        if not asset_id or not category or joints < 0:
            raise ValueError(f"invalid PV-A roster row at ordinal {ordinal}")
        row_text = _canonical_text(row)
        row_digest = hashlib.sha256(row_text.encode("utf-8")).hexdigest()
        values = (
            ordinal,
            asset_id,
            category,
            joints,
            row_digest,
            row_text,
        )
        if inserted < existing_assets:
            observed = existing_rows.fetchone()
            if observed != values:
                raise ValueError(f"partial result database roster mismatch at {ordinal}")
        else:
            batch.append(values)
        inserted += 1
        j_eval += joints
        categories.add(category)
        if len(batch) >= 1_000:
            connection.executemany(
                "INSERT INTO assets(ordinal, asset_id, category, joint_count, row_sha256, row_json) "
                "VALUES(?, ?, ?, ?, ?, ?)",
                batch,
            )
            connection.commit()
            batch.clear()
    if batch:
        connection.executemany(
            "INSERT INTO assets(ordinal, asset_id, category, joint_count, row_sha256, row_json) "
            "VALUES(?, ?, ?, ?, ?, ?)",
            batch,
        )
        connection.commit()
    if inserted != selected_n:
        raise ValueError(f"PV-A selected roster count mismatch: {inserted} != {selected_n}")
    if existing_rows.fetchone() is not None:
        raise ValueError("partial result database contains a non-prefix roster row")
    _set_meta(
        connection,
        {
            **binding,
            "roster_manifest": str(roster_manifest_path),
            "selected_j": j_eval,
            "selected_category_count": len(categories),
            "asset_import_state": "COMPLETE",
        },
    )
    connection.commit()
    return {
        "manifest": manifest,
        "n_eval": selected_n,
        "j_eval": j_eval,
        "category_count": len(categories),
    }


def _execution_manifest(
    roster_path: Path,
    roster: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
    category_count: int,
    workers: int,
    timeout_seconds: float,
    run_standard_parser: bool,
    limit: int | None,
    protocol_binding: Mapping[str, str],
) -> dict[str, Any]:
    value: dict[str, Any] = {
        "schema_version": RUN_SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "classification": (
            "SMOKE"
            if limit is not None
            else "NON_FORMAL_DIAGNOSTIC"
            if not run_standard_parser
            else str(roster.get("classification", "CUSTOM_COHORT"))
        ),
        "roster_manifest": str(roster_path),
        "roster_manifest_sha256": common.sha256_file(roster_path),
        "roster_manifest_content_sha256": roster["manifest_content_sha256"],
        "N_eval": n_eval,
        "J_eval": j_eval,
        "category_count": category_count,
        "limit": limit,
        "workers": workers,
        "asset_timeout_seconds": timeout_seconds,
        "run_standard_parser": run_standard_parser,
        "fresh_interpreter_per_asset": True,
        "protocol": {
            **dict(protocol_binding),
            "table3_samples": 21,
            "table4_single_joint_samples": table4.SINGLE_SAMPLES,
            "table4_sobol_samples": table4.SOBOL_SAMPLES,
            "table4_sobol_seed": table4.SOBOL_SEED,
            "table4_penetration_threshold_m": table4.PENETRATION_THRESHOLD_M,
        },
        "source_hashes": _source_hashes(),
        "runtime_identity": table4.runtime_identity(),
        "started_at_utc": utc_now(),
    }
    value["manifest_content_sha256"] = _self_hash(value, "manifest_content_sha256")
    return value


def _table2sup_job(row: Mapping[str, Any]) -> dict[str, Any]:
    package = Path(str(row["source_path"])).resolve(strict=True)
    primary = Path(str(row["primary_urdf_path"])).resolve(strict=True)
    try:
        relative = primary.relative_to(package).as_posix()
    except ValueError as error:
        raise ValueError("PV-A primary URDF escapes source package") from error
    return {
        "selection_index": int(row["ordinal"]),
        "asset_id": str(row["asset_id"]),
        "category": str(row.get("raw_category", row.get("category", ""))),
        "package": str(package),
        "primary_urdf_relative_path": relative,
        "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
        "expected_movable_joints": int(row.get("joint_count", 0)),
    }


def _frozen_table2sup_job(row: Mapping[str, Any]) -> dict[str, Any]:
    package = Path(str(row.get("source_path", "")))
    primary = Path(str(row.get("primary_urdf_path", "")))
    try:
        relative = primary.relative_to(package).as_posix()
    except ValueError:
        relative = str(row.get("primary_urdf_relative_path", "model.urdf"))
    return {
        "selection_index": int(row.get("ordinal", 0)),
        "asset_id": str(row.get("asset_id", "")),
        "category": str(row.get("raw_category", row.get("category", ""))),
        "package": str(package),
        "primary_urdf_relative_path": relative,
        "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
        "expected_movable_joints": int(row.get("joint_count", 0)),
    }


def _table4_job(row: Mapping[str, Any]) -> dict[str, Any]:
    dof = int(row.get("joint_count", 0))
    job: dict[str, Any] = {
        "protocol_id": "urdf_sim_ready_table4_pva_full_release_v1",
        "order": int(row.get("ordinal", 0)),
        "dataset": "pva",
        "dataset_id": str(row.get("asset_id", "")),
        "asset_id": str(row.get("asset_id", "")),
        "category": str(row.get("category") or row.get("raw_category") or ""),
        "package": str(row.get("source_path", "")),
        "urdf_path": str(row.get("primary_urdf_path", "")),
        "primary_urdf_path": str(row.get("primary_urdf_path", "")),
        "primary_urdf_relative_path": str(
            row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
        "expected_movable_joints": dof,
        "rest_state_expected": 1,
        "single_state_expected": table4.SINGLE_SAMPLES * dof,
        "sobol_state_expected": table4.SOBOL_SAMPLES if dof else 0,
        "package_binding_sha256": row.get("package_binding_sha256"),
        "roster_ordinal": int(row.get("ordinal", 0)),
    }
    job["input_identity_sha256"] = table4.canonical_sha256(
        {
            key: job[key]
            for key in (
                "dataset",
                "dataset_id",
                "category",
                "urdf_path",
                "primary_urdf_relative_path",
                "expected_primary_urdf_sha256",
                "expected_movable_joints",
                "package_binding_sha256",
            )
        }
    )
    return job


def _write_child_result(path: Path, record: Mapping[str, Any]) -> None:
    _atomic_json(path, dict(record))


def _bind_record_source(
    table_name: str,
    record: Mapping[str, Any],
    row: Mapping[str, Any],
    *,
    package_verified: bool,
) -> dict[str, Any]:
    """Attach the frozen roster atoms used by the independent checker."""

    bound = dict(record)
    ordinal = int(row["ordinal"])
    asset_id = str(row["asset_id"])
    category = str(row.get("raw_category", row.get("category", "")))
    primary_hash = row.get("primary_urdf_sha256")
    package_hash = row.get("package_binding_sha256")
    bound["package_binding_verified"] = bool(package_verified)
    bound["package_binding_sha256"] = package_hash
    if table_name in {"table1", "table2"}:
        bound.update(
            {
                "asset_id": asset_id,
                "ordinal": ordinal,
                "raw_category": category,
                "primary_urdf_sha256": primary_hash,
            }
        )
        if table_name == "table1":
            bound["roster_joint_count"] = int(row.get("joint_count", 0))
    elif table_name == "table2_supplementary":
        bound.update(
            {
                "asset_id": asset_id,
                "selection_index": ordinal,
                "ordinal": ordinal,
                "category": category,
                "expected_primary_urdf_sha256": primary_hash,
                "expected_movable_joints": int(row.get("joint_count", 0)),
            }
        )
    elif table_name == "table3":
        bound.update(
            {
                "asset_id": asset_id,
                "asset_key": asset_id,
                "ordinal": ordinal,
                "category": category,
                "primary_urdf_sha256": primary_hash,
                "roster_joint_count": int(row.get("joint_count", 0)),
                "sample_count_expected": 21,
            }
        )
    elif table_name == "table4":
        job = _table4_job(row)
        bound.update(
            {
                "dataset_id": asset_id,
                "order": ordinal,
                "category": category,
                "expected_primary_urdf_sha256": primary_hash,
                "expected_movable_joints": int(row.get("joint_count", 0)),
                "input_identity_sha256": job["input_identity_sha256"],
            }
        )
    else:
        raise ValueError(f"unknown PV-A result table: {table_name}")
    return bound


def _valid_child_result(path: Path, table_name: str, row: Mapping[str, Any]) -> bool:
    if not path.is_file():
        return False
    try:
        value = _json(path)
    except ValueError:
        return False
    expected = str(row["asset_id"])
    observed = value.get("dataset_id") if table_name == "table4" else value.get("asset_id")
    return str(observed) == expected


def _child_run(job_path: Path, child_root: Path) -> int:
    payload = _json(job_path)
    row = payload.get("row")
    if not isinstance(row, dict):
        raise ValueError("child job has no roster row")
    run_standard_parser = bool(payload.get("run_standard_parser", True))
    child_root.mkdir(parents=True, exist_ok=True)
    os.environ.update(BLAS_ENVIRONMENT)

    try:
        pva_roster._verify_frozen_package(row)
    except BaseException as error:  # noqa: BLE001 - retain the asset everywhere
        reason = f"package_binding_preflight: {type(error).__name__}: {error}"
        failures = _failure_records(row, reason)
        for name, record in failures.items():
            _write_child_result(child_root / f"{name}.json", record)
        _atomic_json(
            child_root / "completion.json",
            {"asset_id": row["asset_id"], "ordinal": row["ordinal"]},
        )
        return 0

    table1_path = child_root / "table1.json"
    if not _valid_child_result(table1_path, "table1", row):
        try:
            record1 = table1.audit_row(row, run_standard_parser)
        except BaseException as error:  # noqa: BLE001
            record1 = table1.failure_record(row, f"child_exception: {type(error).__name__}: {error}")
        record1 = _bind_record_source(
            "table1", record1, row, package_verified=True
        )
        _write_child_result(table1_path, record1)

    table2_path = child_root / "table2.json"
    if not _valid_child_result(table2_path, "table2", row):
        try:
            record2 = table2.audit_row(row, run_standard_parser)
        except BaseException as error:  # noqa: BLE001
            record2 = table2.failure_record(row, f"child_exception: {type(error).__name__}: {error}")
        record2 = _bind_record_source(
            "table2", record2, row, package_verified=True
        )
        _write_child_result(table2_path, record2)

    table2sup_path = child_root / "table2_supplementary.json"
    if not _valid_child_result(table2sup_path, "table2_supplementary", row):
        job2s = _table2sup_job(row)
        try:
            package = Path(job2s["package"])
            record2s = table2sup_static.audit_package(
                package,
                urdf_relative_path=str(job2s["primary_urdf_relative_path"]),
                asset_id=str(job2s["asset_id"]),
                expected_movable_joints=int(job2s["expected_movable_joints"]),
                placeholder_registry=[],
            )
            record2s.update(
                {
                    "category": job2s["category"],
                    "selection_index": job2s["selection_index"],
                    "ordinal": int(row["ordinal"]),
                    "result_origin": "child_audit",
                    "expected_primary_urdf_sha256": job2s["expected_primary_urdf_sha256"],
                    "expected_movable_joints": job2s["expected_movable_joints"],
                }
            )
        except BaseException as error:  # noqa: BLE001
            record2s = table2sup._failure(
                job2s,
                f"child_exception: {type(error).__name__}: {error}",
            )
            record2s["ordinal"] = int(row["ordinal"])
        record2s = _bind_record_source(
            "table2_supplementary", record2s, row, package_verified=True
        )
        _write_child_result(table2sup_path, record2s)

    table3_path = child_root / "table3.json"
    if not _valid_child_result(table3_path, "table3", row):
        try:
            record3 = table3.evaluate_row(
                row,
                samples=21,
                scratch_root=child_root / "table3_scratch",
            )
        except BaseException as error:  # noqa: BLE001
            record3 = table3._failed(
                row,
                f"child_exception: {type(error).__name__}: {error}",
                samples=21,
            )
            record3.update(
                {
                    "ordinal": int(row["ordinal"]),
                    "category": row.get("raw_category", row.get("category")),
                    "primary_urdf_sha256": row.get("primary_urdf_sha256"),
                    "roster_joint_count": int(row.get("joint_count", 0)),
                    "sample_count_expected": 21,
                }
            )
        record3 = _bind_record_source(
            "table3", record3, row, package_verified=True
        )
        _write_child_result(table3_path, record3)

    table4_path = child_root / "table4.json"
    if not _valid_child_result(table4_path, "table4", row):
        job4 = _table4_job(row)
        try:
            record4 = table4.evaluate_job(job4)
        except BaseException as error:  # noqa: BLE001
            record4 = table4._empty_record(
                job4, f"child_exception: {type(error).__name__}: {error}"
            )
        record4 = _bind_record_source(
            "table4", record4, row, package_verified=True
        )
        _write_child_result(table4_path, record4)
    _atomic_json(child_root / "completion.json", {"asset_id": row["asset_id"], "ordinal": row["ordinal"]})
    return 0


def _failure_records(row: Mapping[str, Any], reason: str) -> dict[str, dict[str, Any]]:
    row_dict = dict(row)
    ordinal = int(row_dict["ordinal"])
    record1 = table1.failure_record(row_dict, reason)
    record1["ordinal"] = ordinal
    record2 = table2.failure_record(row_dict, reason)
    record2["ordinal"] = ordinal
    job2s = _frozen_table2sup_job(row_dict)
    record2s = table2sup._failure(job2s, reason)
    record2s["ordinal"] = ordinal
    record3 = table3._failed(row_dict, reason, samples=21)
    record3.update(
        {
            "ordinal": ordinal,
            "category": row_dict.get("raw_category", row_dict.get("category")),
            "primary_urdf_sha256": row_dict.get("primary_urdf_sha256"),
            "roster_joint_count": int(row_dict.get("joint_count", 0)),
            "sample_count_expected": 21,
        }
    )
    record4 = table4._empty_record(_table4_job(row_dict), reason)
    records = {
        "table1": record1,
        "table2": record2,
        "table2_supplementary": record2s,
        "table3": record3,
        "table4": record4,
    }
    return {
        name: _bind_record_source(name, record, row_dict, package_verified=False)
        for name, record in records.items()
    }


def _collect_child_results(
    child_root: Path,
    row: Mapping[str, Any],
    *,
    fallback_reason: str,
) -> dict[str, Any]:
    failures = _failure_records(row, fallback_reason)
    paths = {
        "table1": child_root / "table1.json",
        "table2": child_root / "table2.json",
        "table2_supplementary": child_root / "table2_supplementary.json",
        "table3": child_root / "table3.json",
        "table4": child_root / "table4.json",
    }
    results: dict[str, dict[str, Any]] = {}
    expected_id = str(row["asset_id"])
    for name, path in paths.items():
        try:
            record = _json(path)
            observed = record.get("dataset_id") if name == "table4" else record.get("asset_id")
            if str(observed) != expected_id:
                raise ValueError("child result identity mismatch")
            results[name] = record
        except (OSError, ValueError, json.JSONDecodeError):
            results[name] = failures[name]
    states = results["table4"].pop("state_records", [])
    if not isinstance(states, list) or any(not isinstance(item, Mapping) for item in states):
        results["table4"] = failures["table4"]
        states = []
    state_payload = b"".join(
        (_canonical_text(dict(item)) + "\n").encode("utf-8") for item in states
    )
    return {
        "records": results,
        "states_zlib": zlib.compress(state_payload, level=6),
        "state_count": len(states),
    }


def _kill_group(process: subprocess.Popen[Any]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except (OSError, AttributeError):
        try:
            process.terminate()
        except OSError:
            return
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except (OSError, AttributeError):
            try:
                process.kill()
            except OSError:
                pass
        process.wait()


def _acquire_asset_lock(path: Path, timeout_seconds: float) -> int | None:
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_RDWR | os.O_CREAT
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags, 0o600)
    deadline = time.monotonic() + timeout_seconds
    try:
        while True:
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return descriptor
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    os.close(descriptor)
                    return None
                time.sleep(min(0.05, remaining))
    except BaseException:
        os.close(descriptor)
        raise


def _release_asset_lock(descriptor: int) -> None:
    try:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        os.close(descriptor)


def _execute_asset(
    row: Mapping[str, Any],
    output: Path,
    *,
    timeout_seconds: float,
    run_standard_parser: bool,
) -> dict[str, Any]:
    started = time.monotonic()
    ordinal = int(row["ordinal"])
    lock_path = (
        output
        / "asset_locks"
        / f"{ordinal // 1_000:06d}"
        / f"{ordinal:09d}.lock"
    )
    lock_descriptor = _acquire_asset_lock(lock_path, timeout_seconds)
    if lock_descriptor is None:
        return _terminal_payload(
            row,
            f"asset_lock_timeout_after_{timeout_seconds:g}_seconds",
            time.monotonic() - started,
            worker_status="lock_timeout",
        )

    child_root = output / "children" / f"{ordinal:09d}"
    process: subprocess.Popen[Any] | None = None
    transferred = False
    try:
        child_root.mkdir(parents=True, exist_ok=True)
        job_path = child_root / "job.json"
        job = {"row": dict(row), "run_standard_parser": run_standard_parser}
        if job_path.is_file():
            try:
                if _json(job_path) != job:
                    raise ValueError("stale child job binding")
            except ValueError:
                shutil.rmtree(child_root)
                child_root.mkdir(parents=True)
        _atomic_json(job_path, job)
        completed = child_root / "completion.json"
        status = "recovered"
        fallback = "child result unavailable"
        if not completed.is_file():
            environment = dict(os.environ)
            environment.update(BLAS_ENVIRONMENT)
            log_path = child_root / "worker.log"
            with log_path.open("ab") as log:
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(SCRIPT),
                        "--child",
                        "--job",
                        str(job_path),
                        "--child-output",
                        str(child_root),
                    ],
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=environment,
                    start_new_session=True,
                    pass_fds=(lock_descriptor,),
                )
                try:
                    code = process.wait(timeout=timeout_seconds)
                except subprocess.TimeoutExpired:
                    _kill_group(process)
                    code = None
            if code is None:
                status = "timeout"
                fallback = f"asset_timeout_after_{timeout_seconds:g}_seconds"
            elif code != 0:
                status = "child_error"
                try:
                    detail = log_path.read_text(encoding="utf-8", errors="replace")[-2_000:]
                except OSError:
                    detail = ""
                fallback = f"child_exit_{code}: {detail}"
            else:
                status = "completed"
                fallback = "child completed without a valid table result"
        collected = _collect_child_results(child_root, row, fallback_reason=fallback)
        records = collected["records"]
        if any(
            str(record.get("result_origin", "")).startswith("parent_synthesized")
            for record in records.values()
        ):
            status = "partial_fail_closed"
        payload = {
            "ordinal": ordinal,
            "asset_id": str(row["asset_id"]),
            "table1_json": _canonical_text(records["table1"]),
            "table2_json": _canonical_text(records["table2"]),
            "table2sup_json": _canonical_text(records["table2_supplementary"]),
            "table3_json": _canonical_text(records["table3"]),
            "table4_json": _canonical_text(records["table4"]),
            "table4_states_zlib": collected["states_zlib"],
            "table4_state_count": int(collected["state_count"]),
            "worker_status": status,
            "worker_seconds": time.monotonic() - started,
            "completed_at_utc": utc_now(),
            "child_root": child_root,
            "asset_lock_descriptor": lock_descriptor,
        }
        transferred = True
        return payload
    finally:
        if not transferred:
            if process is not None and process.poll() is None:
                _kill_group(process)
            _release_asset_lock(lock_descriptor)


def _terminal_payload(
    row: Mapping[str, Any],
    reason: str,
    elapsed: float,
    *,
    worker_status: str = "parent_error",
) -> dict[str, Any]:
    failures = _failure_records(row, reason)
    return {
        "ordinal": int(row["ordinal"]),
        "asset_id": str(row["asset_id"]),
        "table1_json": _canonical_text(failures["table1"]),
        "table2_json": _canonical_text(failures["table2"]),
        "table2sup_json": _canonical_text(failures["table2_supplementary"]),
        "table3_json": _canonical_text(failures["table3"]),
        "table4_json": _canonical_text(failures["table4"]),
        "table4_states_zlib": zlib.compress(b""),
        "table4_state_count": 0,
        "worker_status": worker_status,
        "worker_seconds": elapsed,
        "completed_at_utc": utc_now(),
        "child_root": None,
    }


def _validated_result_prefix(connection: sqlite3.Connection) -> int:
    expected_ordinal = 0
    rows = connection.execute(
        "SELECT r.ordinal, r.asset_id, a.asset_id "
        "FROM results AS r LEFT JOIN assets AS a ON a.ordinal = r.ordinal "
        "ORDER BY r.ordinal"
    )
    for ordinal, result_asset_id, roster_asset_id in rows:
        if int(ordinal) != expected_ordinal:
            raise ValueError(
                "result database is not a contiguous prefix: "
                f"expected ordinal {expected_ordinal}, observed {ordinal}"
            )
        if roster_asset_id is None or str(result_asset_id) != str(roster_asset_id):
            raise ValueError(
                f"result database contiguous prefix identity mismatch at {expected_ordinal}"
            )
        expected_ordinal += 1
    return expected_ordinal


def _pending_rows(
    connection: sqlite3.Connection,
    *,
    start_ordinal: int,
    limit: int,
) -> list[dict[str, Any]]:
    if start_ordinal < 0 or limit <= 0:
        raise ValueError("pending-row cursor and limit must be positive")
    rows: list[dict[str, Any]] = []
    query = (
        "SELECT ordinal, row_json FROM assets "
        "WHERE ordinal >= ? ORDER BY ordinal LIMIT ?"
    )
    for offset, (ordinal, payload) in enumerate(
        connection.execute(query, (start_ordinal, limit))
    ):
        expected = start_ordinal + offset
        if int(ordinal) != expected:
            raise ValueError(
                f"asset roster is not contiguous at pending ordinal {expected}"
            )
        value = json.loads(payload)
        if not isinstance(value, dict):
            raise ValueError(f"asset roster row {expected} is not an object")
        rows.append(value)
    return rows


def _insert_result(connection: sqlite3.Connection, payload: Mapping[str, Any]) -> None:
    connection.execute(
        """
        INSERT INTO results(
            ordinal, asset_id, table1_json, table2_json, table2sup_json,
            table3_json, table4_json, table4_states_zlib,
            table4_state_count, worker_status, worker_seconds, completed_at_utc
        ) VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            payload["ordinal"],
            payload["asset_id"],
            payload["table1_json"],
            payload["table2_json"],
            payload["table2sup_json"],
            payload["table3_json"],
            payload["table4_json"],
            payload["table4_states_zlib"],
            payload["table4_state_count"],
            payload["worker_status"],
            payload["worker_seconds"],
            payload["completed_at_utc"],
        ),
    )


def _write_progress(
    output: Path,
    connection: sqlite3.Connection,
    *,
    n_eval: int,
    j_eval: int,
    started: float,
    status: str,
) -> None:
    completed, states = connection.execute(
        "SELECT COUNT(*), COALESCE(SUM(table4_state_count), 0) FROM results"
    ).fetchone()
    worker_counts = dict(
        connection.execute(
            "SELECT worker_status, COUNT(*) FROM results GROUP BY worker_status ORDER BY worker_status"
        )
    )
    _atomic_json(
        output / "progress.json",
        {
            "schema_version": "pva_table1234_progress_v1",
            "status": status,
            "completed_assets": int(completed),
            "total_assets": n_eval,
            "J_eval": j_eval,
            "table4_state_records": int(states),
            "worker_status_counts": worker_counts,
            "elapsed_seconds": time.monotonic() - started,
            "updated_at_utc": utc_now(),
        },
    )


def _result_records(connection: sqlite3.Connection, column: str) -> Iterator[dict[str, Any]]:
    allowed = set(RESULT_COLUMNS.values())
    if column not in allowed:
        raise ValueError(f"unknown result column: {column}")
    for (payload,) in connection.execute(f"SELECT {column} FROM results ORDER BY ordinal"):
        value = json.loads(payload)
        if not isinstance(value, dict):
            raise ValueError(f"non-object result in {column}")
        yield value


def _minimal_roster_rows(connection: sqlite3.Connection) -> Iterator[dict[str, Any]]:
    query = (
        "SELECT ordinal, asset_id, category, joint_count, row_json "
        "FROM assets ORDER BY ordinal"
    )
    for ordinal, asset_id, category, joint_count, payload in connection.execute(query):
        source = json.loads(payload)
        if not isinstance(source, dict):
            raise ValueError(f"non-object roster row at ordinal {ordinal}")
        raw_category = str(source.get("raw_category", category))
        yield {
            "ordinal": int(ordinal),
            "asset_id": str(asset_id),
            "raw_category": raw_category,
            "category": str(source.get("category", raw_category)),
            "joint_count": int(joint_count),
            "primary_urdf_sha256": source.get("primary_urdf_sha256"),
        }


def _artifact_manifest(root: Path, names: Sequence[str]) -> dict[str, Any]:
    entries = []
    for name in sorted(names):
        path = root / name
        entries.append(
            {"path": name, "size": path.stat().st_size, "sha256": common.sha256_file(path)}
        )
    value: dict[str, Any] = {
        "schema_version": common.ARTIFACT_SCHEMA_VERSION,
        "artifacts": entries,
    }
    value["artifact_manifest_content_sha256"] = common.canonical_sha256(value)
    return value


def _publish_table(
    output: Path,
    name: str,
    records: list[dict[str, Any]],
    summary: dict[str, Any],
    *,
    state_writer: Any = None,
) -> dict[str, Any]:
    table_root = output / name
    table_root.mkdir(parents=True, exist_ok=True)
    records_name = "records.jsonl" if name in {"table3", "table4", "table2_supplementary"} else "asset_records.jsonl"
    count = _atomic_jsonl(table_root / records_name, records)
    summary = dict(summary)
    summary["summary_content_sha256"] = _self_hash(summary, "summary_content_sha256")
    _atomic_json(table_root / "summary.json", summary)
    state_count = None
    artifacts = [records_name, "summary.json", "checkpoint.json"]
    if state_writer is not None:
        state_count = int(state_writer(table_root / "state_records.jsonl"))
        artifacts.append("state_records.jsonl")
    checkpoint: dict[str, Any] = {
        "schema_version": "pva_table_checkpoint_v1",
        "state": "complete",
        "records": count,
        "N_eval": int(summary.get("n_eval", summary.get("N_eval", summary.get("cohort", {}).get("N_eval", count)))),
        "J_eval": int(summary.get("j_eval", 0)),
        "records_sha256": common.sha256_file(table_root / records_name),
        "summary_sha256": common.sha256_file(table_root / "summary.json"),
    }
    if state_count is not None:
        checkpoint["state_records"] = state_count
        checkpoint["state_records_sha256"] = common.sha256_file(table_root / "state_records.jsonl")
    checkpoint["checkpoint_content_sha256"] = _self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    _atomic_json(table_root / "checkpoint.json", checkpoint)
    artifact_manifest_path = table_root / "artifact_manifest.json"
    artifact_manifest = _artifact_manifest(table_root, artifacts)
    _atomic_json(artifact_manifest_path, artifact_manifest)
    return {
        "records": str((table_root / records_name).relative_to(output)),
        "records_count": count,
        "records_sha256": common.sha256_file(table_root / records_name),
        "summary": str((table_root / "summary.json").relative_to(output)),
        "summary_sha256": common.sha256_file(table_root / "summary.json"),
        "checkpoint": str((table_root / "checkpoint.json").relative_to(output)),
        "checkpoint_sha256": common.sha256_file(table_root / "checkpoint.json"),
        "artifact_manifest": str(artifact_manifest_path.relative_to(output)),
        "artifact_manifest_sha256": common.sha256_file(artifact_manifest_path),
        "artifact_manifest_content_sha256": artifact_manifest[
            "artifact_manifest_content_sha256"
        ],
        **(
            {
                "state_records": str((table_root / "state_records.jsonl").relative_to(output)),
                "state_records_count": state_count,
                "state_records_sha256": common.sha256_file(table_root / "state_records.jsonl"),
            }
            if state_count is not None
            else {}
        ),
    }


def _write_states(connection: sqlite3.Connection, path: Path) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    count = 0
    try:
        with os.fdopen(fd, "wb") as stream:
            for blob, declared_count in connection.execute(
                "SELECT table4_states_zlib, table4_state_count FROM results ORDER BY ordinal"
            ):
                payload = zlib.decompress(blob)
                observed = payload.count(b"\n")
                if observed != int(declared_count):
                    raise ValueError("compressed Table 4 state count mismatch")
                stream.write(payload)
                count += observed
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
    return count


def _finalize(
    connection: sqlite3.Connection,
    output: Path,
    roster_manifest_path: Path,
    roster_manifest: Mapping[str, Any],
    execution_manifest: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
    category_count: int,
) -> dict[str, Any]:
    result_count = int(connection.execute("SELECT COUNT(*) FROM results").fetchone()[0])
    if result_count != n_eval:
        raise ValueError(f"cannot finalize incomplete PV-A run: {result_count} != {n_eval}")
    rows = list(_minimal_roster_rows(connection))
    pseudo_roster = {
        "schema_version": common.SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "N_eval": n_eval,
        "J_eval": j_eval,
        "rows": rows,
        "roster_sha256": roster_manifest["roster"]["sha256"],
        "manifest_content_sha256": roster_manifest["manifest_content_sha256"],
    }
    published: dict[str, dict[str, Any]] = {}

    records = list(_result_records(connection, RESULT_COLUMNS["table1"]))
    summary1 = table1.aggregate_full_release(records, pseudo_roster)
    summary1["cohort"]["cohort_type"] = "FULL_LOCAL_RELEASE"
    summary1["cohort"]["N_release"] = int(roster_manifest["N_release"])
    summary1["cohort"]["release_raw_categories"] = int(roster_manifest["release_category_count"])
    published["table1"] = _publish_table(output, "table1", records, summary1)
    del records, summary1

    records = list(_result_records(connection, RESULT_COLUMNS["table2"]))
    summary2 = table2.aggregate_full_release(records, pseudo_roster)
    published["table2"] = _publish_table(output, "table2", records, summary2)
    del records, summary2

    records = list(
        _result_records(connection, RESULT_COLUMNS["table2_supplementary"])
    )
    summary2s = table2sup.aggregate_records(records, n_eval, j_eval)
    summary2s.update(
        {
            "schema_version": "pva_table2_supplementary_full_release_v1",
            "dataset": "Ours / PV-A",
            "category_count": category_count,
        }
    )
    published["table2_supplementary"] = _publish_table(
        output,
        "table2_supplementary",
        records,
        summary2s,
    )
    del records, summary2s

    records = list(_result_records(connection, RESULT_COLUMNS["table3"]))
    summary3 = table3.aggregate_full_release(records, pseudo_roster)
    published["table3"] = _publish_table(output, "table3", records, summary3)
    del records, summary3

    records = list(_result_records(connection, RESULT_COLUMNS["table4"]))
    summary4 = table4.aggregate_records(records, n_eval, j_eval)
    summary4.update(
        {
            "dataset": "Ours / PV-A",
            "dataset_slug": "pva",
            "category_count": category_count,
        }
    )
    published["table4"] = _publish_table(
        output,
        "table4",
        records,
        summary4,
        state_writer=lambda path: _write_states(connection, path),
    )
    del records, summary4
    receipt: dict[str, Any] = {
        "schema_version": RECEIPT_SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "classification": execution_manifest["classification"],
        "N_release": int(roster_manifest["N_release"]),
        "N_eval": n_eval,
        "J_eval": j_eval,
        "release_category_count": int(roster_manifest["release_category_count"]),
        "eval_category_count": category_count,
        "roster_manifest": str(roster_manifest_path),
        "roster_manifest_sha256": common.sha256_file(roster_manifest_path),
        "roster_manifest_content_sha256": roster_manifest["manifest_content_sha256"],
        "execution_manifest": "manifest.json",
        "execution_manifest_sha256": common.sha256_file(output / "manifest.json"),
        "tables": published,
        "result_database": "results.sqlite3",
        "completed_at_utc": utc_now(),
    }
    return receipt


def _seal_receipt(output: Path, receipt: dict[str, Any]) -> Path:
    database = output / "results.sqlite3"
    receipt["result_database_bytes"] = database.stat().st_size
    receipt["result_database_sha256"] = common.sha256_file(database)
    receipt["receipt_content_sha256"] = _self_hash(receipt, "receipt_content_sha256")
    path = output / "full_release_receipt.json"
    _atomic_json(path, receipt)
    return path


def run_full_release(
    roster_manifest_path: Path,
    output: Path,
    *,
    workers: int = 32,
    timeout_seconds: float = 600.0,
    run_standard_parser: bool = True,
    resume: bool = False,
    limit: int | None = None,
    batch_size: int | None = None,
) -> Path:
    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    roster_manifest_path = Path(roster_manifest_path).resolve(strict=True)
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()) and not resume:
        raise ValueError(f"evaluation output is not empty; use --resume: {output}")
    output.mkdir(parents=True, exist_ok=True)
    protocol_binding = _freeze_protocol(output)
    started = time.monotonic()
    database_path = output / "results.sqlite3"
    connection = _connect(database_path)
    receipt: dict[str, Any]
    try:
        _create_schema(connection)
        initialized = _initialize_assets(connection, roster_manifest_path, limit=limit)
        roster_manifest = initialized["manifest"]
        n_eval = int(initialized["n_eval"])
        j_eval = int(initialized["j_eval"])
        category_count = int(initialized["category_count"])
        requested_manifest = _execution_manifest(
            roster_manifest_path,
            roster_manifest,
            n_eval=n_eval,
            j_eval=j_eval,
            category_count=category_count,
            workers=workers,
            timeout_seconds=timeout_seconds,
            run_standard_parser=run_standard_parser,
            limit=limit,
            protocol_binding=protocol_binding,
        )
        manifest_path = output / "manifest.json"
        if manifest_path.is_file():
            existing_manifest = _json(manifest_path)
            if existing_manifest.get("manifest_content_sha256") != _self_hash(
                existing_manifest, "manifest_content_sha256"
            ):
                raise ValueError("execution manifest self-hash mismatch")
            for key in (
                "schema_version",
                "roster_manifest_sha256",
                "roster_manifest_content_sha256",
                "N_eval",
                "J_eval",
                "category_count",
                "limit",
                "workers",
                "asset_timeout_seconds",
                "run_standard_parser",
                "source_hashes",
                "runtime_identity",
                "protocol",
            ):
                if existing_manifest.get(key) != requested_manifest.get(key):
                    raise ValueError(f"resume configuration mismatch: {key}")
            execution_manifest = existing_manifest
        else:
            execution_manifest = requested_manifest
            _atomic_json(manifest_path, execution_manifest)

        batch_size = batch_size or max(workers * 4, workers)
        if batch_size < workers:
            raise ValueError("batch_size cannot be smaller than workers")
        next_ordinal = _validated_result_prefix(connection)
        if next_ordinal > n_eval:
            raise ValueError("result database prefix exceeds N_eval")
        _write_progress(
            output,
            connection,
            n_eval=n_eval,
            j_eval=j_eval,
            started=started,
            status="RUNNING",
        )
        while True:
            pending = _pending_rows(
                connection,
                start_ordinal=next_ordinal,
                limit=batch_size,
            )
            if not pending:
                break
            payloads: list[dict[str, Any]] = []
            committed = False
            try:
                with ThreadPoolExecutor(max_workers=workers) as executor:
                    futures = {
                        executor.submit(
                            _execute_asset,
                            row,
                            output,
                            timeout_seconds=timeout_seconds,
                            run_standard_parser=run_standard_parser,
                        ): row
                        for row in pending
                    }
                    for future in as_completed(futures):
                        row = futures[future]
                        asset_started = time.monotonic()
                        try:
                            payload = future.result()
                        except BaseException as error:  # noqa: BLE001
                            payload = _terminal_payload(
                                row,
                                f"parent_executor_exception: {type(error).__name__}: {error}",
                                time.monotonic() - asset_started,
                            )
                        payloads.append(payload)
                payloads.sort(key=lambda item: int(item["ordinal"]))
                expected_ordinals = list(
                    range(next_ordinal, next_ordinal + len(pending))
                )
                observed_ordinals = [int(item["ordinal"]) for item in payloads]
                if observed_ordinals != expected_ordinals:
                    raise ValueError("worker payloads do not preserve the pending prefix")
                try:
                    connection.execute("BEGIN")
                    for payload in payloads:
                        _insert_result(connection, payload)
                    connection.commit()
                    committed = True
                except BaseException:
                    connection.rollback()
                    raise
            finally:
                for payload in payloads:
                    descriptor = payload.pop("asset_lock_descriptor", None)
                    if committed:
                        child_root = payload.get("child_root")
                        if isinstance(child_root, Path):
                            shutil.rmtree(child_root, ignore_errors=True)
                    if isinstance(descriptor, int):
                        _release_asset_lock(descriptor)
            next_ordinal += len(payloads)
            _write_progress(
                output,
                connection,
                n_eval=n_eval,
                j_eval=j_eval,
                started=started,
                status="RUNNING",
            )

        receipt = _finalize(
            connection,
            output,
            roster_manifest_path,
            roster_manifest,
            execution_manifest,
            n_eval=n_eval,
            j_eval=j_eval,
            category_count=category_count,
        )
        _write_progress(
            output,
            connection,
            n_eval=n_eval,
            j_eval=j_eval,
            started=started,
            status="COMPLETE",
        )
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        connection.commit()
    finally:
        connection.close()
    children = output / "children"
    if children.is_dir():
        try:
            children.rmdir()
        except OSError:
            pass
    _seal_receipt(output, receipt)
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roster", type=Path, default=DEFAULT_ROSTER)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=32)
    parser.add_argument("--timeout-seconds", type=float, default=600.0)
    parser.add_argument("--no-standard-parser", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--job", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--child-output", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.child:
        if args.job is None or args.child_output is None:
            parser.error("--child requires --job and --child-output")
        try:
            return _child_run(args.job, args.child_output)
        except BaseException as error:  # noqa: BLE001
            print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
            return 1
    try:
        result = run_full_release(
            args.roster,
            args.output,
            workers=args.workers,
            timeout_seconds=args.timeout_seconds,
            run_standard_parser=not args.no_standard_parser,
            resume=args.resume,
            limit=args.limit,
            batch_size=args.batch_size,
        )
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "COMPLETE", "output": str(result)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["RECEIPT_SCHEMA_VERSION", "RUN_SCHEMA_VERSION", "run_full_release"]
