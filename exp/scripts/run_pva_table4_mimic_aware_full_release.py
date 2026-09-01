#!/usr/bin/env python3
"""Rerun PV-A Table 4 with the mimic-aware v2 sampling protocol.

The sealed Table 1--4 PV-A result database is used only as an indexed copy of
the frozen roster.  Results are written to a separate Table4-only database, so
the historical independent-joint v1 evidence is never modified.  Each asset is
still evaluated in a fresh interpreter by ``run_table4_full_release.py``.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import sys
import tempfile
import time
from typing import Any, Iterable, Iterator, Mapping
import zlib


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import build_pva_full_release_roster as pva_roster
import run_pva_table1234_full_release as pva_run
import run_table4_full_release as table4


RUN_SCHEMA_VERSION = "pva_table4_mimic_aware_full_release_run_v2"
RESULT_DB_SCHEMA_VERSION = "pva_table4_mimic_aware_results_db_v1"
RECEIPT_SCHEMA_VERSION = "pva_table4_mimic_aware_full_release_receipt_v1"
PACKAGE_ROOT_BINDING_SCHEMA_VERSION = "pva_package_root_override_v1"
SAMPLING_PROTOCOL = table4.SAMPLING_PROTOCOL_V2
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v2"
DEFAULT_SOURCE_EVALUATION = (
    SCRIPT.parents[1]
    / "runtime"
    / "pva_table1234_full_release_20260826"
    / "evaluation"
)
DEFAULT_OUTPUT = (
    SCRIPT.parents[1]
    / "runtime"
    / "pva_table4_mimic_aware_full_release_20260827"
)
PROTOCOL_DOCUMENT = SCRIPT.parents[1] / "URDF-Sim-Ready-Automatic-Evaluation.md"


def _canonical_text(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_text(value).encode("utf-8")).hexdigest()


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    payload = dict(value)
    payload.pop(field, None)
    return _canonical_sha256(payload)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, value: Any) -> None:
    pva_run._atomic_json(path, value)


def _atomic_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> int:
    return pva_run._atomic_jsonl(path, rows)


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _connect_output(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(path, timeout=120.0)
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA synchronous=FULL")
    connection.execute("PRAGMA temp_store=MEMORY")
    return connection


def _connect_source(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(f"{path.resolve().as_uri()}?mode=ro", uri=True)
    connection.execute("PRAGMA query_only=ON")
    return connection


def _create_schema(connection: sqlite3.Connection) -> None:
    connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS meta (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS results (
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            record_json TEXT NOT NULL,
            states_zlib BLOB NOT NULL,
            state_count INTEGER NOT NULL CHECK (state_count >= 0),
            worker_status TEXT NOT NULL,
            worker_seconds REAL NOT NULL CHECK (worker_seconds >= 0),
            completed_at_utc TEXT NOT NULL
        );
        """
    )
    connection.commit()


def _decoded_meta(connection: sqlite3.Connection) -> dict[str, Any]:
    return {
        str(key): json.loads(value)
        for key, value in connection.execute("SELECT key, value FROM meta")
    }


def _set_meta(connection: sqlite3.Connection, values: Mapping[str, Any]) -> None:
    connection.executemany(
        "INSERT OR REPLACE INTO meta(key, value) VALUES(?, ?)",
        [(str(key), _canonical_text(value)) for key, value in values.items()],
    )
    connection.commit()


def _source_context(source_evaluation: Path) -> tuple[dict[str, Any], sqlite3.Connection]:
    source_evaluation = Path(source_evaluation).resolve(strict=True)
    receipt_path = source_evaluation / "full_release_receipt.json"
    database_path = source_evaluation / "results.sqlite3"
    receipt = _load_json(receipt_path)
    if receipt.get("schema_version") != pva_run.RECEIPT_SCHEMA_VERSION:
        raise ValueError("source evaluation is not a sealed PV-A Table 1--4 release")
    if receipt.get("receipt_content_sha256") != _self_hash(
        receipt, "receipt_content_sha256"
    ):
        raise ValueError("source PV-A receipt self-hash mismatch")
    if not database_path.is_file():
        raise ValueError("source PV-A result database is missing")

    source = _connect_source(database_path)
    try:
        meta = _decoded_meta(source)
        expected = {
            "schema_version": pva_run.RESULT_DB_SCHEMA_VERSION,
            "asset_import_state": "COMPLETE",
            "selected_n": int(receipt["N_eval"]),
            "selected_j": int(receipt["J_eval"]),
            "selected_category_count": int(receipt["eval_category_count"]),
            "roster_manifest_sha256": receipt["roster_manifest_sha256"],
            "roster_manifest_content_sha256": receipt[
                "roster_manifest_content_sha256"
            ],
        }
        for key, value in expected.items():
            if meta.get(key) != value:
                raise ValueError(f"source PV-A database binding mismatch: {key}")
        count, joints, categories, first_ordinal, last_ordinal = source.execute(
            "SELECT COUNT(*), COALESCE(SUM(joint_count), 0), "
            "COUNT(DISTINCT category), MIN(ordinal), MAX(ordinal) FROM assets"
        ).fetchone()
        if (
            int(count) != expected["selected_n"]
            or int(joints) != expected["selected_j"]
            or int(categories) != expected["selected_category_count"]
            or (int(count) and (int(first_ordinal) != 0 or int(last_ordinal) != int(count) - 1))
        ):
            raise ValueError("source PV-A asset index is incomplete")
    except BaseException:
        source.close()
        raise

    context = {
        "source_evaluation": str(source_evaluation),
        "source_receipt": str(receipt_path),
        "source_receipt_sha256": _sha256_file(receipt_path),
        "source_receipt_content_sha256": receipt["receipt_content_sha256"],
        "source_result_database": str(database_path),
        # The sealed receipt owns this 11 GB database digest.  Rechecking every
        # resume would add minutes without strengthening the frozen row hashes.
        "source_result_database_declared_sha256": receipt["result_database_sha256"],
        "roster_manifest": str(Path(receipt["roster_manifest"]).resolve()),
        "roster_manifest_sha256": receipt["roster_manifest_sha256"],
        "roster_manifest_content_sha256": receipt[
            "roster_manifest_content_sha256"
        ],
        "N_eval": int(receipt["N_eval"]),
        "J_eval": int(receipt["J_eval"]),
        "category_count": int(receipt["eval_category_count"]),
    }
    return context, source


def _source_hashes() -> dict[str, str]:
    return {
        "adapter": _sha256_file(SCRIPT),
        "table4_runner": _sha256_file(Path(table4.__file__)),
        "table4_core": _sha256_file(Path(table4.CORE_SCRIPT)),
        "pva_roster_verifier": _sha256_file(Path(pva_roster.__file__)),
    }


def _package_root_binding(
    context: Mapping[str, Any], package_root_override: Path | None
) -> dict[str, Any] | None:
    if package_root_override is None:
        return None
    roster_path = Path(str(context["roster_manifest"])).resolve(strict=True)
    if _sha256_file(roster_path) != context["roster_manifest_sha256"]:
        raise ValueError("source roster file hash mismatch before mirror binding")
    roster = _load_json(roster_path)
    if roster.get("schema_version") != pva_roster.ROSTER_SCHEMA_VERSION:
        raise ValueError("source roster schema mismatch before mirror binding")
    if roster.get("manifest_content_sha256") != _self_hash(
        roster, "manifest_content_sha256"
    ):
        raise ValueError("source roster self-hash mismatch before mirror binding")
    if (
        roster.get("manifest_content_sha256")
        != context["roster_manifest_content_sha256"]
    ):
        raise ValueError("source roster content binding mismatch before mirror binding")
    source_bindings = roster.get("source_bindings")
    if not isinstance(source_bindings, Mapping):
        raise ValueError("source roster has no source bindings for mirror mapping")
    logical_value = source_bindings.get("extracted_root")
    if not isinstance(logical_value, str) or not logical_value:
        raise ValueError("source roster has no frozen extracted root")
    logical_root = Path(logical_value)
    if not logical_root.is_absolute() or ".." in logical_root.parts:
        raise ValueError("frozen logical package root is not a safe absolute path")

    requested_root = Path(package_root_override)
    if not requested_root.is_absolute():
        requested_root = Path.cwd() / requested_root
    pva_run.common._assert_no_symlink(requested_root)
    physical_root = requested_root.resolve(strict=True)
    if not physical_root.is_dir():
        raise ValueError(f"package root override is not a directory: {physical_root}")
    value: dict[str, Any] = {
        "schema_version": PACKAGE_ROOT_BINDING_SCHEMA_VERSION,
        "mapping_policy": "relative_prefix_substitution_v1",
        "logical_root": str(logical_root),
        "physical_root": str(physical_root),
        "roster_manifest_content_sha256": context[
            "roster_manifest_content_sha256"
        ],
        "package_verification": "frozen_package_files_sha256_v1",
    }
    value["binding_content_sha256"] = _self_hash(
        value, "binding_content_sha256"
    )
    return value


def _mapped_execution_paths(
    row: Mapping[str, Any], package_root_binding: Mapping[str, Any]
) -> dict[str, str]:
    if (
        package_root_binding.get("schema_version")
        != PACKAGE_ROOT_BINDING_SCHEMA_VERSION
        or package_root_binding.get("mapping_policy")
        != "relative_prefix_substitution_v1"
        or package_root_binding.get("binding_content_sha256")
        != _self_hash(package_root_binding, "binding_content_sha256")
    ):
        raise ValueError("package root override binding is invalid")
    logical_root = Path(str(package_root_binding["logical_root"]))
    physical_root = Path(str(package_root_binding["physical_root"]))
    logical_package = Path(str(row.get("source_path", "")))
    logical_urdf = Path(str(row.get("primary_urdf_path", "")))
    if not logical_package.is_absolute() or not logical_urdf.is_absolute():
        raise ValueError("frozen package and URDF paths must be absolute")
    try:
        package_relative = logical_package.relative_to(logical_root)
        urdf_relative = logical_urdf.relative_to(logical_package)
    except ValueError as error:
        raise ValueError("frozen package or URDF escapes the logical package root") from error
    if (
        package_relative == Path(".")
        or urdf_relative == Path(".")
        or ".." in package_relative.parts
        or ".." in urdf_relative.parts
    ):
        raise ValueError("frozen package or URDF has an unsafe relative mapping")
    evaluation_package = physical_root / package_relative
    evaluation_urdf = evaluation_package / urdf_relative
    return {
        "evaluation_package_path": str(evaluation_package),
        "evaluation_urdf_path": str(evaluation_urdf),
        "evaluation_package_relative_path": package_relative.as_posix(),
        "evaluation_urdf_relative_path": urdf_relative.as_posix(),
    }


def _verified_execution_paths(
    row: Mapping[str, Any], package_root_binding: Mapping[str, Any] | None
) -> dict[str, str] | None:
    if package_root_binding is None:
        pva_roster._verify_frozen_package(row)
        return None
    paths = _mapped_execution_paths(row, package_root_binding)
    physical_root = Path(str(package_root_binding["physical_root"]))
    evaluation_package = Path(paths["evaluation_package_path"])
    evaluation_urdf = Path(paths["evaluation_urdf_path"])
    for path in (physical_root, evaluation_package, evaluation_urdf):
        pva_run.common._assert_no_symlink(path)
    physical_root = physical_root.resolve(strict=True)
    evaluation_package = evaluation_package.resolve(strict=True)
    evaluation_urdf = evaluation_urdf.resolve(strict=True)
    try:
        evaluation_package.relative_to(physical_root)
        evaluation_urdf.relative_to(evaluation_package)
    except ValueError as error:
        raise ValueError("resolved mirror path escapes the physical package root") from error
    if not evaluation_package.is_dir() or not evaluation_urdf.is_file():
        raise ValueError("mapped mirror package or primary URDF has the wrong type")
    physical_row = dict(row)
    physical_row["source_path"] = str(evaluation_package)
    pva_roster._verify_frozen_package(physical_row)
    paths["evaluation_package_path"] = str(evaluation_package)
    paths["evaluation_urdf_path"] = str(evaluation_urdf)
    return paths


def _freeze_protocol(output: Path) -> dict[str, Any]:
    source = PROTOCOL_DOCUMENT.resolve(strict=True)
    snapshot = output / "protocol_snapshot.md"
    if snapshot.exists():
        if not snapshot.is_file() or snapshot.is_symlink():
            raise ValueError("protocol snapshot is not a regular file")
    else:
        pva_run.common._atomic_write_bytes(snapshot, source.read_bytes())
    return {
        "source_document": str(source),
        "snapshot": snapshot.relative_to(output).as_posix(),
        "snapshot_sha256": _sha256_file(snapshot),
    }


def _manifest(
    context: Mapping[str, Any],
    *,
    output: Path,
    workers: int,
    timeout_seconds: float,
    limit: int | None,
    protocol_binding: Mapping[str, Any],
    package_root_binding: Mapping[str, Any] | None,
) -> dict[str, Any]:
    n_eval = int(limit if limit is not None else context["N_eval"])
    value: dict[str, Any] = {
        "schema_version": RUN_SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "dataset_slug": "pva",
        "classification": "SMOKE" if limit is not None else "FORMAL_FULL_RELEASE",
        "N_eval": n_eval,
        "J_eval": None if limit is not None else int(context["J_eval"]),
        "category_count": None if limit is not None else int(context["category_count"]),
        "limit": limit,
        "sampling_protocol": SAMPLING_PROTOCOL,
        "protocol_id": PROTOCOL_ID,
        "protocol": {
            **dict(protocol_binding),
            "single_joint_samples": table4.SINGLE_SAMPLES,
            "sobol_samples": table4.SOBOL_SAMPLES,
            "sobol_seed": table4.SOBOL_SEED,
            "penetration_threshold_m": table4.PENETRATION_THRESHOLD_M,
            "mimic_constraints": "affine_expansion_from_independent_roots",
            "contact_policy": "all_non_direct-parent_pairs",
        },
        "source": dict(context),
        "source_hashes": _source_hashes(),
        "runtime_identity": table4.runtime_identity(),
        "package_root_binding": (
            dict(package_root_binding) if package_root_binding is not None else None
        ),
        "execution": {
            "workers": workers,
            "timeout_seconds": timeout_seconds,
            "fresh_interpreter_per_asset": True,
            "batch_persistence": True,
        },
        "output": str(output),
        "started_at_utc": pva_run.utc_now(),
    }
    value["manifest_content_sha256"] = _self_hash(
        value, "manifest_content_sha256"
    )
    return value


def _initialize_output_database(
    connection: sqlite3.Connection, manifest: Mapping[str, Any]
) -> None:
    binding = {
        "schema_version": RESULT_DB_SCHEMA_VERSION,
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "source_receipt_content_sha256": manifest["source"][
            "source_receipt_content_sha256"
        ],
        "source_result_database_declared_sha256": manifest["source"][
            "source_result_database_declared_sha256"
        ],
        "roster_manifest_content_sha256": manifest["source"][
            "roster_manifest_content_sha256"
        ],
        "N_eval": manifest["N_eval"],
        "J_eval": manifest["J_eval"],
        "limit": manifest["limit"],
        "sampling_protocol": SAMPLING_PROTOCOL,
        "protocol_id": PROTOCOL_ID,
        "package_root_binding_content_sha256": (
            manifest.get("package_root_binding") or {}
        ).get("binding_content_sha256"),
    }
    observed = _decoded_meta(connection)
    if observed:
        for key, value in binding.items():
            if observed.get(key) != value:
                raise ValueError(f"result database resume binding mismatch: {key}")
    else:
        _set_meta(connection, binding)


def _selected_totals(
    source: sqlite3.Connection, n_eval: int
) -> tuple[int, int]:
    joints, categories = source.execute(
        "SELECT COALESCE(SUM(joint_count), 0), COUNT(DISTINCT category) "
        "FROM assets WHERE ordinal < ?",
        (n_eval,),
    ).fetchone()
    return int(joints), int(categories)


@lru_cache(maxsize=1)
def _collision_core() -> Any:
    return table4._core()


def _job_with_plan(
    row: Mapping[str, Any],
    plan: Mapping[str, Any],
    *,
    package_root_binding: Mapping[str, Any] | None = None,
    execution_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    dof = int(row.get("joint_count", 0))
    package = Path(str(row.get("source_path", ""))).resolve()
    urdf = Path(str(row.get("primary_urdf_path", ""))).resolve()
    independent = int(plan["independent_dof_count"])
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
        **plan,
        "rest_state_expected": 1,
        "single_state_expected": table4.SINGLE_SAMPLES * independent,
        "sobol_state_expected": table4.SOBOL_SAMPLES if independent else 0,
        "package_binding_sha256": row.get("package_binding_sha256"),
        "roster_ordinal": int(row["ordinal"]),
    }
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
        "joint_sampling_plan_sha256",
        "single_state_expected",
        "sobol_state_expected",
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
                else _mapped_execution_paths(row, package_root_binding)
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
        execution_paths = _verified_execution_paths(row, package_root_binding)
    urdf = Path(
        str(
            execution_paths["evaluation_urdf_path"]
            if execution_paths is not None
            else row.get("primary_urdf_path", "")
        )
    ).resolve()
    plan = _collision_core().sampling_plan_metadata(
        urdf,
        declared_dof=int(row.get("joint_count", 0)),
        expected_sha256=row.get("primary_urdf_sha256"),
    )
    return _job_with_plan(
        row,
        plan,
        package_root_binding=package_root_binding,
        execution_paths=execution_paths,
    )


def _fallback_job(
    row: Mapping[str, Any],
    issue: str,
    package_root_binding: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Freeze a conservative denominator without parsing an unverified package."""

    dof = int(row.get("joint_count", 0))
    return _job_with_plan(
        row,
        {
            "independent_dof_count": dof,
            "range_evaluable_independent_dof_count": 0,
            "mimic_joint_count": 0,
            "fixed_root_joint_count": 0,
            "joint_sampling_plan_sha256": None,
            "sampling_plan_error": issue,
        },
        package_root_binding=package_root_binding,
    )


def _validate_result(
    result: Mapping[str, Any], job: Mapping[str, Any]
) -> dict[str, Any]:
    if str(result.get("dataset_id")) != str(job["dataset_id"]):
        raise ValueError("child result asset identity mismatch")
    if int(result.get("order", -1)) != int(job["order"]):
        raise ValueError("child result ordinal mismatch")
    for field in (
        "protocol_id",
        "sampling_protocol",
        "package",
        "urdf_path",
        "input_identity_sha256",
        "package_root_binding_content_sha256",
        "execution_input_sha256",
        "evaluation_package_relative_path",
        "evaluation_urdf_relative_path",
        "expected_primary_urdf_sha256",
        "joint_sampling_plan_sha256",
        "independent_dof_count",
        "range_evaluable_independent_dof_count",
        "mimic_joint_count",
        "fixed_root_joint_count",
        "single_state_expected",
        "sobol_state_expected",
    ):
        if result.get(field) != job.get(field):
            raise ValueError(f"child result binding mismatch: {field}")
    states = result.get("state_records")
    if not isinstance(states, list) or any(not isinstance(state, Mapping) for state in states):
        raise ValueError("child result state payload is invalid")
    if int(result.get("state_records_count", len(states))) != len(states):
        raise ValueError("child result state count mismatch")
    if result.get("state_records_sha256") != table4.canonical_sha256(states):
        raise ValueError("child result state hash mismatch")
    return dict(result)


def _recover_child(output: Path, job: Mapping[str, Any]) -> dict[str, Any] | None:
    path = output / "children" / f"{int(job['order']):06d}.json"
    if not path.is_file():
        return None
    try:
        return _validate_result(_load_json(path), job)
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _evaluate_row(
    row: Mapping[str, Any],
    output: Path,
    timeout_seconds: float,
    package_root_binding: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    started = time.monotonic()
    job: dict[str, Any] | None = None
    package_verified = False
    worker_status = "completed"
    try:
        execution_paths = _verified_execution_paths(row, package_root_binding)
        package_verified = True
        job = _build_job(
            row,
            package_root_binding=package_root_binding,
            execution_paths=execution_paths,
        )
        result = _recover_child(output, job)
        if result is not None:
            worker_status = "recovered"
        else:
            result = table4._execute_child(job, output, timeout_seconds)
            result = _validate_result(result, job)
            if str(result.get("status")) == "timeout":
                worker_status = "timeout"
    except BaseException as error:  # retain every frozen asset fail-closed
        if not package_verified:
            job = _fallback_job(
                row,
                f"package_preflight_failed: {type(error).__name__}: {error}",
                package_root_binding,
            )
        elif job is None:
            job = _fallback_job(
                row,
                f"job_freeze_failed: {type(error).__name__}: {error}",
                package_root_binding,
            )
        assert job is not None
        result = table4._empty_record(
            job,
            f"pva_v2_worker_exception: {type(error).__name__}: {error}",
        )
        result["state_records_count"] = 0
        worker_status = "package_error" if not package_verified else "worker_error"

    result.update(
        {
            "dataset_id": str(row["asset_id"]),
            "order": int(row["ordinal"]),
            "category": str(row.get("raw_category", row.get("category", ""))),
            "package_binding_verified": package_verified,
            "package_binding_sha256": row.get("package_binding_sha256"),
            "roster_ordinal": int(row["ordinal"]),
        }
    )
    states = result.pop("state_records", [])
    payload = b"".join(
        (_canonical_text(dict(state)) + "\n").encode("utf-8") for state in states
    )
    return {
        "ordinal": int(row["ordinal"]),
        "asset_id": str(row["asset_id"]),
        "record_json": _canonical_text(result),
        "states_zlib": zlib.compress(payload, level=6),
        "state_count": len(states),
        "worker_status": worker_status,
        "worker_seconds": time.monotonic() - started,
        "completed_at_utc": pva_run.utc_now(),
    }


def _validated_result_prefix(
    result: sqlite3.Connection, source: sqlite3.Connection
) -> int:
    expected = 0
    for ordinal, asset_id in result.execute(
        "SELECT ordinal, asset_id FROM results ORDER BY ordinal"
    ):
        if int(ordinal) != expected:
            raise ValueError(
                f"result database is not a contiguous prefix at ordinal {expected}"
            )
        source_row = source.execute(
            "SELECT asset_id FROM assets WHERE ordinal = ?", (expected,)
        ).fetchone()
        if source_row is None or str(source_row[0]) != str(asset_id):
            raise ValueError(f"result/source identity mismatch at ordinal {expected}")
        expected += 1
    return expected


def _pending_rows(
    source: sqlite3.Connection, *, start_ordinal: int, limit: int, n_eval: int
) -> list[dict[str, Any]]:
    if start_ordinal < 0 or limit <= 0:
        raise ValueError("pending-row cursor and limit must be positive")
    count = min(limit, n_eval - start_ordinal)
    rows: list[dict[str, Any]] = []
    query = (
        "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
        "FROM assets WHERE ordinal >= ? AND ordinal < ? ORDER BY ordinal"
    )
    for offset, values in enumerate(
        source.execute(query, (start_ordinal, start_ordinal + count))
    ):
        ordinal, asset_id, category, joint_count, row_hash, payload = values
        expected = start_ordinal + offset
        if int(ordinal) != expected:
            raise ValueError(f"source asset index is not contiguous at {expected}")
        row = json.loads(payload)
        if not isinstance(row, dict):
            raise ValueError(f"source roster row {expected} is not an object")
        if _canonical_sha256(row) != str(row_hash):
            raise ValueError(f"source roster row hash mismatch at {expected}")
        if (
            int(row.get("ordinal", -1)) != expected
            or str(row.get("asset_id")) != str(asset_id)
            or str(row.get("raw_category", row.get("category", ""))) != str(category)
            or int(row.get("joint_count", -1)) != int(joint_count)
        ):
            raise ValueError(f"source roster row binding mismatch at {expected}")
        rows.append(row)
    if len(rows) != count:
        raise ValueError("source asset index ended before N_eval")
    return rows


def _insert_result(connection: sqlite3.Connection, payload: Mapping[str, Any]) -> None:
    connection.execute(
        "INSERT INTO results(ordinal, asset_id, record_json, states_zlib, "
        "state_count, worker_status, worker_seconds, completed_at_utc) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?)",
        (
            payload["ordinal"],
            payload["asset_id"],
            payload["record_json"],
            payload["states_zlib"],
            payload["state_count"],
            payload["worker_status"],
            payload["worker_seconds"],
            payload["completed_at_utc"],
        ),
    )


def _result_records(connection: sqlite3.Connection) -> Iterator[dict[str, Any]]:
    for (payload,) in connection.execute(
        "SELECT record_json FROM results ORDER BY ordinal"
    ):
        value = json.loads(payload)
        if not isinstance(value, dict):
            raise ValueError("result database contains a non-object record")
        yield value


def _write_states(connection: sqlite3.Connection, path: Path) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent)
    )
    count = 0
    try:
        with os.fdopen(descriptor, "wb") as stream:
            for blob, declared in connection.execute(
                "SELECT states_zlib, state_count FROM results ORDER BY ordinal"
            ):
                payload = zlib.decompress(blob)
                observed = payload.count(b"\n")
                if observed != int(declared):
                    raise ValueError("compressed state count mismatch")
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
        "SELECT COUNT(*), COALESCE(SUM(state_count), 0) FROM results"
    ).fetchone()
    worker_counts = dict(
        connection.execute(
            "SELECT worker_status, COUNT(*) FROM results "
            "GROUP BY worker_status ORDER BY worker_status"
        )
    )
    _atomic_json(
        output / "progress.json",
        {
            "schema_version": "pva_table4_mimic_aware_progress_v1",
            "status": status,
            "completed_assets": int(completed),
            "total_assets": n_eval,
            "J_eval": j_eval,
            "state_records": int(states),
            "worker_status_counts": worker_counts,
            "elapsed_seconds": time.monotonic() - started,
            "updated_at_utc": pva_run.utc_now(),
        },
    )


def _cleanup_children(output: Path, ordinals: Iterable[int]) -> None:
    for ordinal in ordinals:
        name = f"{int(ordinal):06d}"
        for directory, suffix in (
            ("child_jobs", ".json"),
            ("children", ".json"),
            ("child_logs", ".log"),
        ):
            (output / directory / f"{name}{suffix}").unlink(missing_ok=True)


def _artifact_manifest(output: Path) -> dict[str, Any]:
    artifacts = []
    for name in (
        "manifest.json",
        "protocol_snapshot.md",
        "records.jsonl",
        "asset_records.jsonl",
        "state_records.jsonl",
        "summary.json",
        "summary.md",
        "checkpoint.json",
        "results.sqlite3",
    ):
        path = output / name
        if path.is_file():
            artifacts.append(
                {
                    "path": name,
                    "bytes": path.stat().st_size,
                    "sha256": _sha256_file(path),
                }
            )
    value = {
        "schema_version": "pva_table4_mimic_aware_artifacts_v1",
        "artifacts": artifacts,
    }
    value["artifact_manifest_content_sha256"] = _self_hash(
        value, "artifact_manifest_content_sha256"
    )
    return value


def _finalize(
    connection: sqlite3.Connection,
    output: Path,
    manifest: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
    category_count: int,
) -> dict[str, Any]:
    count = int(connection.execute("SELECT COUNT(*) FROM results").fetchone()[0])
    if count != n_eval:
        raise ValueError(f"cannot finalize incomplete run: {count} != {n_eval}")
    records_path = output / "records.jsonl"
    records_count = _atomic_jsonl(records_path, _result_records(connection))
    if records_count != n_eval:
        raise ValueError("published record count mismatch")
    shutil.copyfile(records_path, output / "asset_records.jsonl")
    states_count = _write_states(connection, output / "state_records.jsonl")
    records = list(_result_records(connection))
    summary = table4.aggregate_records(records, n_eval, j_eval)
    independent = sum(int(row.get("independent_dof_count", 0)) for row in records)
    mimic = sum(int(row.get("mimic_joint_count", 0)) for row in records)
    fixed_roots = sum(int(row.get("fixed_root_joint_count", 0)) for row in records)
    range_independent = sum(
        int(row.get("range_evaluable_independent_dof_count", 0)) for row in records
    )
    records_hash = _sha256_file(records_path)
    summary.update(
        {
            "schema_version": "pva_table4_mimic_aware_summary_v2",
            "dataset": "Ours / PV-A",
            "dataset_slug": "pva",
            "category_count": category_count,
            "sampling_protocol": SAMPLING_PROTOCOL,
            "declared_dof_count": j_eval,
            "independent_dof_count": independent,
            "range_evaluable_independent_dof_count": range_independent,
            "mimic_joint_count": mimic,
            "fixed_root_joint_count": fixed_roots,
            "state_records_executed": states_count,
            "records_sha256": records_hash,
            "manifest_content_sha256": manifest["manifest_content_sha256"],
            "completed_at_utc": pva_run.utc_now(),
        }
    )
    summary["summary_content_sha256"] = _self_hash(
        summary, "summary_content_sha256"
    )
    _atomic_json(output / "summary.json", summary)
    pva_run.common._atomic_write_bytes(
        output / "summary.md",
        table4.report_text(summary, "pva").encode("utf-8"),
    )
    checkpoint = {
        "schema_version": "pva_table4_mimic_aware_checkpoint_v1",
        "state": "complete",
        "records": n_eval,
        "state_records": states_count,
        "records_sha256": records_hash,
        "state_records_sha256": _sha256_file(output / "state_records.jsonl"),
        "summary_sha256": _sha256_file(output / "summary.json"),
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "completed_at_utc": pva_run.utc_now(),
    }
    checkpoint["checkpoint_content_sha256"] = _self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    _atomic_json(output / "checkpoint.json", checkpoint)
    return summary


def run_pva_table4_v2(
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
    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    context, source = _source_context(source_evaluation)
    try:
        package_binding = _package_root_binding(context, package_root_override)
    except BaseException:
        source.close()
        raise
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()) and not resume:
        source.close()
        raise ValueError(f"output is not empty; use --resume: {output}")
    output.mkdir(parents=True, exist_ok=True)
    protocol = _freeze_protocol(output)
    n_eval = int(limit if limit is not None else context["N_eval"])
    if n_eval <= 0 or n_eval > int(context["N_eval"]):
        source.close()
        raise ValueError(f"limit must be in [1, {context['N_eval']}]")
    selected_j, selected_categories = _selected_totals(source, n_eval)
    requested_manifest = _manifest(
        context,
        output=output,
        workers=workers,
        timeout_seconds=timeout_seconds,
        limit=limit,
        protocol_binding=protocol,
        package_root_binding=package_binding,
    )
    requested_manifest["J_eval"] = selected_j
    requested_manifest["category_count"] = selected_categories
    requested_manifest["manifest_content_sha256"] = _self_hash(
        requested_manifest, "manifest_content_sha256"
    )
    manifest_path = output / "manifest.json"
    if manifest_path.is_file():
        manifest = _load_json(manifest_path)
        if manifest.get("manifest_content_sha256") != _self_hash(
            manifest, "manifest_content_sha256"
        ):
            source.close()
            raise ValueError("run manifest self-hash mismatch")
        for key in (
            "schema_version",
            "N_eval",
            "J_eval",
            "category_count",
            "limit",
            "sampling_protocol",
            "protocol_id",
            "protocol",
            "source",
            "source_hashes",
            "runtime_identity",
            "package_root_binding",
            "execution",
        ):
            if manifest.get(key) != requested_manifest.get(key):
                source.close()
                raise ValueError(f"resume configuration mismatch: {key}")
    else:
        manifest = requested_manifest
        _atomic_json(manifest_path, manifest)

    started = time.monotonic()
    database_path = output / "results.sqlite3"
    result = _connect_output(database_path)
    try:
        _create_schema(result)
        _initialize_output_database(result, manifest)
        next_ordinal = _validated_result_prefix(result, source)
        batch_size = batch_size or max(workers * 4, workers)
        if batch_size < workers:
            raise ValueError("batch_size cannot be smaller than workers")
        _write_progress(
            output,
            result,
            n_eval=n_eval,
            j_eval=selected_j,
            started=started,
            status="RUNNING",
        )
        while next_ordinal < n_eval:
            rows = _pending_rows(
                source,
                start_ordinal=next_ordinal,
                limit=batch_size,
                n_eval=n_eval,
            )
            payloads: list[dict[str, Any]] = []
            with ThreadPoolExecutor(max_workers=workers) as executor:
                futures = {
                    (
                        executor.submit(_evaluate_row, row, output, timeout_seconds)
                        if package_binding is None
                        else executor.submit(
                            _evaluate_row,
                            row,
                            output,
                            timeout_seconds,
                            package_binding,
                        )
                    ): row
                    for row in rows
                }
                for future in as_completed(futures):
                    row = futures[future]
                    try:
                        payload = future.result()
                    except BaseException as error:
                        job = _fallback_job(
                            row,
                            f"parent_executor_exception: {type(error).__name__}: {error}",
                            package_binding,
                        )
                        record = table4._empty_record(
                            job,
                            f"parent_executor_exception: {type(error).__name__}: {error}",
                        )
                        record["state_records_count"] = 0
                        payload = {
                            "ordinal": int(row["ordinal"]),
                            "asset_id": str(row["asset_id"]),
                            "record_json": _canonical_text(record),
                            "states_zlib": zlib.compress(b""),
                            "state_count": 0,
                            "worker_status": "parent_error",
                            "worker_seconds": 0.0,
                            "completed_at_utc": pva_run.utc_now(),
                        }
                    payloads.append(payload)
            payloads.sort(key=lambda item: int(item["ordinal"]))
            expected = list(range(next_ordinal, next_ordinal + len(rows)))
            if [int(item["ordinal"]) for item in payloads] != expected:
                raise ValueError("worker payloads do not preserve the pending prefix")
            try:
                result.execute("BEGIN")
                for payload in payloads:
                    _insert_result(result, payload)
                result.commit()
            except BaseException:
                result.rollback()
                raise
            _cleanup_children(output, (int(item["ordinal"]) for item in payloads))
            next_ordinal += len(payloads)
            _write_progress(
                output,
                result,
                n_eval=n_eval,
                j_eval=selected_j,
                started=started,
                status="RUNNING",
            )

        summary = _finalize(
            result,
            output,
            manifest,
            n_eval=n_eval,
            j_eval=selected_j,
            category_count=selected_categories,
        )
        _write_progress(
            output,
            result,
            n_eval=n_eval,
            j_eval=selected_j,
            started=started,
            status="COMPLETE",
        )
        result.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        result.commit()
    finally:
        result.close()
        source.close()

    for directory in ("child_jobs", "children", "child_logs"):
        try:
            (output / directory).rmdir()
        except OSError:
            pass
    artifact_manifest = _artifact_manifest(output)
    _atomic_json(output / "artifact_manifest.json", artifact_manifest)
    receipt = {
        "schema_version": RECEIPT_SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "dataset_slug": "pva",
        "classification": manifest["classification"],
        "N_eval": n_eval,
        "J_eval": selected_j,
        "category_count": selected_categories,
        "sampling_protocol": SAMPLING_PROTOCOL,
        "protocol_id": PROTOCOL_ID,
        "source": dict(context),
        "package_root_binding": manifest.get("package_root_binding"),
        "manifest": "manifest.json",
        "manifest_sha256": _sha256_file(output / "manifest.json"),
        "records": "records.jsonl",
        "records_sha256": _sha256_file(output / "records.jsonl"),
        "state_records": "state_records.jsonl",
        "state_records_sha256": _sha256_file(output / "state_records.jsonl"),
        "summary": "summary.json",
        "summary_sha256": _sha256_file(output / "summary.json"),
        "status": summary["status"],
        "metrics": summary["metrics"],
        "artifact_manifest": "artifact_manifest.json",
        "artifact_manifest_sha256": _sha256_file(
            output / "artifact_manifest.json"
        ),
        "result_database": "results.sqlite3",
        "result_database_sha256": _sha256_file(output / "results.sqlite3"),
        "completed_at_utc": pva_run.utc_now(),
    }
    receipt["receipt_content_sha256"] = _self_hash(
        receipt, "receipt_content_sha256"
    )
    _atomic_json(output / "full_release_receipt.json", receipt)
    return output


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
        output = run_pva_table4_v2(
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
    print(_canonical_text({"status": "COMPLETE", "output": str(output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RECEIPT_SCHEMA_VERSION",
    "RESULT_DB_SCHEMA_VERSION",
    "RUN_SCHEMA_VERSION",
    "run_pva_table4_v2",
]
