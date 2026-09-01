#!/usr/bin/env python3
"""Read-only acceptance checker for the default 847-row PV-A v2 overlay.

Run this after the sparse evaluation has no pending rows and before publishing
the overlay.  The checker never modifies the evaluated output; its self-hashed
JSON report is written to stdout.
"""

from __future__ import annotations

import argparse
from collections import Counter
import ctypes
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import stat
import sys
import tempfile
from typing import Any, Mapping
import zlib


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
EXP_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_table4_full_release as table4


RUN_SCHEMA = "pva_table4_v2_sparse_correction_overlay_run_v1"
OVERLAY_DB_SCHEMA = "pva_table4_v2_sparse_correction_db_v1"
REPORT_SCHEMA = "pva_table4_v2_sparse_acceptance_report_v1"
SELF_TEST_SCHEMA = "pva_table4_v2_sparse_acceptance_self_test_v1"

DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime" / "pva_table4_v2_targeted_correction_20260828"
)
EXPECTED_ASSETS = 847
EXPECTED_REASON_COUNTS = {
    "expanded_rest_normalizer": 633,
    "zero_width_fixed_root": 214,
}
ALLOWED_WORKER_STATUSES = {"completed", "recovered"}
PHASES = ("rest", "single", "sobol")


class AcceptanceError(RuntimeError):
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
        raise AcceptanceError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise AcceptanceError(f"{label} is not a JSON object")
    return value


def require_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != self_hash(value, field):
        raise AcceptanceError(f"{label} self-hash mismatch")


def file_binding(path: Path, label: str) -> dict[str, Any]:
    path = Path(path).resolve(strict=True)
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise AcceptanceError(f"{label} is not a regular non-symlink file")
    return {
        "path": str(path),
        "bytes": int(info.st_size),
        "sha256": sha256_file(path),
    }


def verify_binding(binding: Any, label: str) -> Path:
    if not isinstance(binding, Mapping):
        raise AcceptanceError(f"{label} binding is missing")
    path = Path(str(binding.get("path", "")))
    if not path.is_absolute():
        raise AcceptanceError(f"{label} binding path is not absolute")
    if file_binding(path, label) != dict(binding):
        raise AcceptanceError(f"{label} binding mismatch")
    return path.resolve(strict=True)


def runtime_identity_quiet() -> dict[str, Any]:
    """Probe the runtime without contaminating the JSON-only stdout contract."""

    sys.stdout.flush()
    sys.stderr.flush()
    saved_stdout = os.dup(1)
    saved_stderr = os.dup(2)
    null = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(null, 1)
        os.dup2(null, 2)
        value = table4.runtime_identity()
        ctypes.CDLL(None).fflush(None)
    finally:
        os.dup2(saved_stdout, 1)
        os.dup2(saved_stderr, 2)
        os.close(null)
        os.close(saved_stdout)
        os.close(saved_stderr)
    if not isinstance(value, dict):
        raise AcceptanceError("runtime identity probe did not return an object")
    return value


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


def _file_identity(path: Path) -> tuple[int, int, int, int]:
    info = path.lstat()
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise AcceptanceError(f"unsafe SQLite database: {path}")
    return (
        int(info.st_dev),
        int(info.st_ino),
        int(info.st_size),
        int(info.st_mtime_ns),
    )


def _sidecar_snapshot(database: Path) -> dict[str, tuple[int, int, int, int] | None]:
    result: dict[str, tuple[int, int, int, int] | None] = {}
    for suffix in ("-wal", "-journal", "-shm"):
        path = Path(f"{database}{suffix}")
        try:
            identity = _file_identity(path)
        except FileNotFoundError:
            result[suffix] = None
            continue
        if suffix in {"-wal", "-journal"} and identity[2] != 0:
            raise AcceptanceError(
                f"overlay is not quiescent: non-empty SQLite {suffix[1:]} sidecar"
            )
        result[suffix] = identity
    return result


def _connect_quiescent(database: Path) -> tuple[sqlite3.Connection, dict[str, Any]]:
    database = Path(database).resolve(strict=True)
    before_identity = _file_identity(database)
    before_sidecars = _sidecar_snapshot(database)
    before_hash = sha256_file(database)
    connection = sqlite3.connect(
        f"{database.as_uri()}?mode=ro&immutable=1", uri=True, timeout=120.0
    )
    connection.execute("PRAGMA query_only=ON")
    if int(connection.execute("PRAGMA query_only").fetchone()[0]) != 1:
        connection.close()
        raise AcceptanceError("failed to enforce query-only SQLite mode")
    return connection, {
        "path": str(database),
        "bytes": before_identity[2],
        "sha256": before_hash,
        "identity": before_identity,
        "sidecars": before_sidecars,
    }


def _require_database_unchanged(database: Path, snapshot: Mapping[str, Any]) -> None:
    if (
        _file_identity(database) != tuple(snapshot["identity"])
        or _sidecar_snapshot(database) != snapshot["sidecars"]
        or sha256_file(database) != snapshot["sha256"]
    ):
        raise AcceptanceError("overlay database changed during acceptance check")


def _decoded_meta(connection: sqlite3.Connection) -> dict[str, Any]:
    try:
        return {
            str(key): json.loads(value)
            for key, value in connection.execute("SELECT key,value FROM meta")
        }
    except (sqlite3.Error, json.JSONDecodeError) as error:
        raise AcceptanceError(f"invalid overlay metadata: {error}") from error


def _decode_states(blob: bytes, expected_count: int) -> list[dict[str, Any]]:
    try:
        raw = zlib.decompress(blob)
        lines = raw.splitlines()
        states = [json.loads(line) for line in lines]
    except (zlib.error, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AcceptanceError(f"invalid compressed state payload: {error}") from error
    if len(states) != expected_count or any(not isinstance(row, dict) for row in states):
        raise AcceptanceError("compressed state count/type mismatch")
    expected_raw = b"".join(
        (canonical_text(state) + "\n").encode("utf-8") for state in states
    )
    if raw != expected_raw:
        raise AcceptanceError("compressed state stream is not canonical JSONL")
    return states


def _selection_identity(selection: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "parent_ordinal": int(selection["parent_ordinal"]),
        "asset_id": str(selection["asset_id"]),
        "source_row_sha256": str(selection["source_row_sha256"]),
        "selection_reasons": selection["reasons"],
    }


def _validate_selection_row(row: tuple[Any, ...], index: int) -> dict[str, Any]:
    keys = (
        "selection_index",
        "parent_ordinal",
        "asset_id",
        "source_row_sha256",
        "reasons_json",
        "new_scale_pose_sha256",
        "new_plan_sha256",
        "new_independent_dof",
        "new_fixed_root_count",
        "audit_json",
        "row_content_sha256",
    )
    selection = dict(zip(keys, row))
    if int(selection["selection_index"]) != index:
        raise AcceptanceError(f"selection index is not contiguous at {index}")
    try:
        reasons = json.loads(str(selection.pop("reasons_json")))
        audit = json.loads(str(selection["audit_json"]))
    except json.JSONDecodeError as error:
        raise AcceptanceError(f"invalid selection JSON at {index}") from error
    if reasons not in (["expanded_rest_normalizer"], ["zero_width_fixed_root"]):
        raise AcceptanceError(f"unexpected default-847 reason set at selection {index}")
    if not isinstance(audit, dict):
        raise AcceptanceError(f"selection audit is not an object at {index}")
    require_self_hash(audit, "row_content_sha256", f"selection audit {index}")
    checks = {
        "parent_ordinal": int(selection["parent_ordinal"]),
        "asset_id": str(selection["asset_id"]),
        "source_row_sha256": str(selection["source_row_sha256"]),
        "selection_reasons": reasons,
        "new_scale_pose_sha256": str(selection["new_scale_pose_sha256"]),
        "new_plan_sha256": str(selection["new_plan_sha256"]),
        "new_independent_dof": int(selection["new_independent_dof"]),
        "new_fixed_root_count": int(selection["new_fixed_root_count"]),
        "row_content_sha256": str(selection["row_content_sha256"]),
    }
    for key, expected in checks.items():
        if audit.get(key) != expected:
            raise AcceptanceError(f"selection audit/column mismatch at {index}: {key}")
    if audit.get("selected") is not True:
        raise AcceptanceError(f"selection audit is not selected at {index}")
    selection["reasons"] = reasons
    return selection


def _phase_closure(record: Mapping[str, Any], ordinal: int) -> dict[str, int]:
    totals: dict[str, int] = {}
    for phase in PHASES:
        expected = int(record.get(f"{phase}_state_expected", -1))
        executed = int(record.get(f"{phase}_state_executed", -2))
        if expected < 0 or executed != expected:
            raise AcceptanceError(
                f"phase execution is incomplete at ordinal {ordinal}: "
                f"{phase} {executed}/{expected}"
            )
        totals[phase] = expected
    if int(record.get("unexecuted_state_count", -1)) != 0:
        raise AcceptanceError(f"nonzero unexecuted state count at ordinal {ordinal}")
    return totals


def _validate_runtime(
    record: Mapping[str, Any],
    expected_runtime: Mapping[str, Any],
    expected_runtime_sha256: str,
    ordinal: int,
) -> None:
    runtime = record.get("runtime_identity")
    if not isinstance(runtime, Mapping):
        raise AcceptanceError(f"runtime identity is missing at ordinal {ordinal}")
    if canonical_sha256(runtime) != expected_runtime_sha256:
        for field in (
            "python_executable",
            "python_version",
            "pybullet_api_version",
            "pybullet_module",
            "pybullet_module_sha256",
            "scipy_version",
            "runner_sha256",
            "collision_core_sha256",
        ):
            if runtime.get(field) != expected_runtime.get(field):
                raise AcceptanceError(
                    f"runtime identity mismatch at ordinal {ordinal}: {field}"
                )
        raise AcceptanceError(f"runtime identity mismatch at ordinal {ordinal}")


def _validate_result(
    *,
    selection: Mapping[str, Any],
    result_row: tuple[Any, ...],
    expected_runtime: Mapping[str, Any],
    expected_runtime_sha256: str,
    core: Any,
) -> dict[str, Any]:
    ordinal = int(selection["parent_ordinal"])
    asset_id = str(selection["asset_id"])
    (
        result_ordinal,
        result_asset_id,
        record_json,
        states_zlib,
        state_count,
        worker_status,
        worker_seconds,
        completed_at_utc,
    ) = result_row
    if int(result_ordinal) != ordinal or str(result_asset_id) != asset_id:
        raise AcceptanceError(f"result identity mismatch at ordinal {ordinal}")
    if str(worker_status) not in ALLOWED_WORKER_STATUSES:
        raise AcceptanceError(f"non-success worker status at ordinal {ordinal}")
    if not math.isfinite(float(worker_seconds)) or float(worker_seconds) < 0.0:
        raise AcceptanceError(f"invalid worker duration at ordinal {ordinal}")
    if not isinstance(completed_at_utc, str) or not completed_at_utc:
        raise AcceptanceError(f"missing completion timestamp at ordinal {ordinal}")
    try:
        record = json.loads(str(record_json))
    except json.JSONDecodeError as error:
        raise AcceptanceError(f"invalid result JSON at ordinal {ordinal}") from error
    if not isinstance(record, dict) or str(record_json) != canonical_text(record):
        raise AcceptanceError(f"result JSON is not canonical at ordinal {ordinal}")
    states = _decode_states(bytes(states_zlib), int(state_count))
    if (
        int(record.get("order", -1)) != ordinal
        or str(record.get("dataset_id", "")) != asset_id
        or record.get("joint_sampling_plan_sha256")
        != selection["new_plan_sha256"]
        or int(record.get("independent_dof_count", -1))
        != int(selection["new_independent_dof"])
        or int(record.get("fixed_root_joint_count", -1))
        != int(selection["new_fixed_root_count"])
        or int(record.get("state_records_count", -1)) != len(states)
        or record.get("state_records_sha256") != canonical_sha256(states)
    ):
        raise AcceptanceError(f"record/selection binding mismatch at ordinal {ordinal}")
    if record.get("status") != "completed":
        raise AcceptanceError(f"record status is not completed at ordinal {ordinal}")
    if record.get("measurement_complete") is not True:
        raise AcceptanceError(f"measurement is incomplete at ordinal {ordinal}")
    if record.get("package_binding_verified") is not True:
        raise AcceptanceError(f"package binding is not verified at ordinal {ordinal}")
    phase_counts = _phase_closure(record, ordinal)
    if len(states) != sum(phase_counts.values()):
        raise AcceptanceError(f"state total does not close at ordinal {ordinal}")
    _validate_runtime(record, expected_runtime, expected_runtime_sha256, ordinal)
    if (
        record.get("normalization_configuration") != "expanded_rest"
        or record.get("normalization_joint_values_sha256")
        != selection["new_scale_pose_sha256"]
    ):
        raise AcceptanceError(f"normalization receipt mismatch at ordinal {ordinal}")
    scale = float(record.get("object_bbox_diagonal_m", math.nan))
    if not math.isfinite(scale) or scale <= 0.0:
        raise AcceptanceError(f"invalid normalization scale at ordinal {ordinal}")
    try:
        core.validate_state_closure(record, states)
    except BaseException as error:
        raise AcceptanceError(
            f"state closure failed at ordinal {ordinal}: {type(error).__name__}: {error}"
        ) from error
    if not states:
        raise AcceptanceError(f"completed record has no states at ordinal {ordinal}")
    maximum = max(float(state["metric_max_penetration_m"]) for state in states)
    if (
        not math.isfinite(maximum)
        or maximum < 0.0
        or not math.isclose(
            maximum,
            float(record.get("max_penetration_m", math.nan)),
            rel_tol=0.0,
            abs_tol=1e-15,
        )
        or not math.isclose(
            maximum / scale,
            float(record.get("max_penetration_normalized", math.nan)),
            rel_tol=1e-12,
            abs_tol=1e-15,
        )
    ):
        raise AcceptanceError(f"normalization arithmetic mismatch at ordinal {ordinal}")
    return {
        "ordinal": ordinal,
        "asset_id": asset_id,
        "record_sha256": hashlib.sha256(str(record_json).encode("utf-8")).hexdigest(),
        "state_records_sha256": record["state_records_sha256"],
        "state_count": len(states),
        "worker_status": str(worker_status),
        "phase_counts": phase_counts,
    }


def check_acceptance(
    output: Path,
    *,
    expected_assets: int = EXPECTED_ASSETS,
    expected_reason_counts: Mapping[str, int] = EXPECTED_REASON_COUNTS,
) -> dict[str, Any]:
    output = Path(output).resolve(strict=True)
    manifest_path = output / "manifest.json"
    manifest = load_json(manifest_path, "correction manifest")
    if manifest.get("schema_version") != RUN_SCHEMA:
        raise AcceptanceError("correction manifest schema mismatch")
    require_self_hash(manifest, "manifest_content_sha256", "correction manifest")
    if Path(str(manifest.get("output", ""))).resolve(strict=True) != output:
        raise AcceptanceError("manifest output binding mismatch")
    selection_manifest = manifest.get("selection")
    if not isinstance(selection_manifest, Mapping):
        raise AcceptanceError("manifest selection receipt is missing")
    if (
        bool(selection_manifest.get("include_backend_capacity_errors"))
        or bool(selection_manifest.get("high_capacity_runtime_attested"))
    ):
        raise AcceptanceError("checker only accepts the default 847-row path")
    expected_reasons = dict(sorted((str(k), int(v)) for k, v in expected_reason_counts.items()))
    if (
        int(selection_manifest.get("selected_asset_count", -1)) != expected_assets
        or int(selection_manifest.get("corrective_selected_asset_count", -1))
        != expected_assets
        or dict(selection_manifest.get("reason_counts", {})) != expected_reasons
    ):
        raise AcceptanceError("manifest default-847 selection totals mismatch")

    correction_sources = manifest.get("correction_sources")
    if not isinstance(correction_sources, Mapping):
        raise AcceptanceError("manifest correction source bindings are missing")
    for name, binding in correction_sources.items():
        verify_binding(binding, f"correction source {name}")
    runner_path = verify_binding(correction_sources.get("runner"), "bound runner")
    core_path = verify_binding(correction_sources.get("core"), "bound core")
    if runner_path != Path(table4.__file__).resolve(strict=True):
        raise AcceptanceError("imported runner differs from manifest-bound runner")
    if core_path != Path(table4.CORE_SCRIPT).resolve(strict=True):
        raise AcceptanceError("runner core differs from manifest-bound core")

    expected_runtime = runtime_identity_quiet()
    expected_runtime_sha256 = canonical_sha256(expected_runtime)
    if (
        expected_runtime.get("runner_sha256")
        != correction_sources["runner"].get("sha256")
        or expected_runtime.get("collision_core_sha256")
        != correction_sources["core"].get("sha256")
    ):
        raise AcceptanceError("current runtime differs from bound runner/core")
    pybullet_hash = expected_runtime.get("pybullet_module_sha256")
    if not isinstance(pybullet_hash, str) or len(pybullet_hash) != 64:
        raise AcceptanceError("current PyBullet runtime hash is invalid")

    database = output / "overlay.sqlite3"
    declared_database = Path(str(selection_manifest.get("database", "")))
    if declared_database.resolve(strict=True) != database.resolve(strict=True):
        raise AcceptanceError("manifest overlay path binding mismatch")
    connection, database_snapshot = _connect_quiescent(database)
    selection_hasher = CanonicalListHasher()
    result_hasher = CanonicalListHasher()
    reason_counts: Counter[str] = Counter()
    worker_counts: Counter[str] = Counter()
    phase_totals: Counter[str] = Counter()
    state_total = 0
    try:
        if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
            raise AcceptanceError("overlay SQLite integrity check failed")
        foreign_errors = connection.execute("PRAGMA foreign_key_check").fetchall()
        if foreign_errors:
            raise AcceptanceError("overlay SQLite foreign-key check failed")
        meta = _decoded_meta(connection)
        if meta.get("schema_version") != OVERLAY_DB_SCHEMA:
            raise AcceptanceError("overlay metadata schema mismatch")
        if meta.get("run_state") not in {"PREPARED", "COMPLETE"}:
            raise AcceptanceError("overlay run state is not prepared/finalized")
        selected_count = int(
            connection.execute("SELECT COUNT(*) FROM selection").fetchone()[0]
        )
        result_count = int(
            connection.execute("SELECT COUNT(*) FROM results").fetchone()[0]
        )
        if selected_count != expected_assets or result_count != expected_assets:
            raise AcceptanceError(
                f"default overlay is incomplete: selection={selected_count}, "
                f"results={result_count}, expected={expected_assets}"
            )
        core = table4._core()
        query = (
            "SELECT s.selection_index,s.parent_ordinal,s.asset_id,"
            "s.source_row_sha256,s.reasons_json,s.new_scale_pose_sha256,"
            "s.new_plan_sha256,s.new_independent_dof,s.new_fixed_root_count,"
            "s.audit_json,s.row_content_sha256,"
            "r.parent_ordinal,r.asset_id,r.record_json,r.states_zlib,"
            "r.state_count,r.worker_status,r.worker_seconds,r.completed_at_utc "
            "FROM selection s LEFT JOIN results r "
            "ON r.parent_ordinal=s.parent_ordinal ORDER BY s.selection_index"
        )
        for index, row in enumerate(connection.execute(query)):
            selection = _validate_selection_row(tuple(row[:11]), index)
            selection_hasher.add(_selection_identity(selection))
            reason_counts.update(selection["reasons"])
            if row[11] is None:
                raise AcceptanceError(
                    f"selected result is missing at ordinal {selection['parent_ordinal']}"
                )
            accepted = _validate_result(
                selection=selection,
                result_row=tuple(row[11:]),
                expected_runtime=expected_runtime,
                expected_runtime_sha256=expected_runtime_sha256,
                core=core,
            )
            worker_counts[accepted["worker_status"]] += 1
            phase_totals.update(accepted["phase_counts"])
            state_total += int(accepted["state_count"])
            result_hasher.add(
                {
                    key: accepted[key]
                    for key in (
                        "ordinal",
                        "asset_id",
                        "record_sha256",
                        "state_records_sha256",
                        "state_count",
                    )
                }
            )
        orphan_count = int(
            connection.execute(
                "SELECT COUNT(*) FROM results r LEFT JOIN selection s "
                "ON s.parent_ordinal=r.parent_ordinal WHERE s.parent_ordinal IS NULL"
            ).fetchone()[0]
        )
        if orphan_count:
            raise AcceptanceError("overlay contains orphan results")
    finally:
        connection.close()
    _require_database_unchanged(database, database_snapshot)

    selection_sha256, selection_count = selection_hasher.finish()
    results_sha256, accepted_count = result_hasher.finish()
    if (
        selection_count != expected_assets
        or accepted_count != expected_assets
        or dict(sorted(reason_counts.items())) != expected_reasons
        or selection_sha256 != selection_manifest.get("selected_assets_sha256")
    ):
        raise AcceptanceError("accepted selection identity/totals mismatch")
    report: dict[str, Any] = {
        "schema_version": REPORT_SCHEMA,
        "status": "PASS",
        "checked_at_utc": utc_now(),
        "output": str(output),
        "checker": file_binding(SCRIPT, "acceptance checker"),
        "manifest": {
            "path": str(manifest_path.resolve(strict=True)),
            "sha256": sha256_file(manifest_path),
            "manifest_content_sha256": manifest["manifest_content_sha256"],
        },
        "overlay": {
            "path": database_snapshot["path"],
            "bytes": database_snapshot["bytes"],
            "sha256": database_snapshot["sha256"],
            "run_state": meta["run_state"],
        },
        "selection": {
            "asset_count": selection_count,
            "reason_counts": dict(sorted(reason_counts.items())),
            "selected_assets_sha256": selection_sha256,
        },
        "accepted_results": {
            "asset_count": accepted_count,
            "state_count": state_total,
            "phase_state_counts": dict(sorted(phase_totals.items())),
            "worker_status_counts": dict(sorted(worker_counts.items())),
            "accepted_results_sha256": results_sha256,
            "all_status_completed": True,
            "all_measurements_complete": True,
            "all_phase_counts_closed": True,
            "all_normalization_receipts_closed": True,
            "all_state_records_closed": True,
        },
        "runtime_identity": {
            "content": expected_runtime,
            "runtime_identity_sha256": expected_runtime_sha256,
            "all_corrected_records_match": True,
        },
    }
    report["acceptance_content_sha256"] = self_hash(
        report, "acceptance_content_sha256"
    )
    return report


def _fixture_state(asset_id: str, ordinal: int) -> dict[str, Any]:
    return {
        "dataset_id": asset_id,
        "order": ordinal,
        "category": "fixture",
        "phase": "rest",
        "joint_name": None,
        "sample_index": 0,
        "joint_values_sha256": canonical_sha256([]),
        "reset_readback_max_abs_error": 0.0,
        "metric_max_penetration_m": 0.0,
        "all_pair_contact_count": 0,
        "all_pair_illegal_penetration_count": 0,
        "all_pair_max_penetration_m": 0.0,
        "non_adjacent_contact_count": 0,
        "non_adjacent_illegal_penetration_count": 0,
        "non_adjacent_max_penetration_m": 0.0,
    }


def _fixture_record(
    asset_id: str, ordinal: int, runtime: Mapping[str, Any]
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    states = [_fixture_state(asset_id, ordinal)]
    record = {
        "dataset_id": asset_id,
        "order": ordinal,
        "category": "fixture",
        "status": "completed",
        "measurement_complete": True,
        "package_binding_verified": True,
        "joint_sampling_plan_sha256": "fixture-plan",
        "independent_dof_count": 0,
        "fixed_root_joint_count": 0,
        "rest_state_expected": 1,
        "rest_state_executed": 1,
        "single_state_expected": 0,
        "single_state_executed": 0,
        "sobol_state_expected": 0,
        "sobol_state_executed": 0,
        "unexecuted_state_count": 0,
        "rest_non_adjacent_free": 1,
        "single_non_adjacent_free": 0,
        "sobol_non_adjacent_free": 0,
        "state_records_count": 1,
        "state_records_sha256": canonical_sha256(states),
        "normalization_configuration": "expanded_rest",
        "normalization_joint_values_sha256": "fixture-rest",
        "object_bbox_diagonal_m": 1.0,
        "max_penetration_m": 0.0,
        "max_penetration_normalized": 0.0,
        "max_reset_readback_error": 0.0,
        "runtime_identity": dict(runtime),
    }
    return record, states


def _write_fixture(root: Path) -> tuple[Path, dict[str, Any], list[dict[str, Any]]]:
    root.mkdir(parents=True, exist_ok=True)
    runtime = runtime_identity_quiet()
    source_bindings = {
        "runner": file_binding(Path(table4.__file__), "fixture runner"),
        "core": file_binding(Path(table4.CORE_SCRIPT), "fixture core"),
    }
    asset_id = "fixture-asset"
    audit = {
        "parent_ordinal": 0,
        "asset_id": asset_id,
        "source_row_sha256": "fixture-source",
        "selection_reasons": ["expanded_rest_normalizer"],
        "new_scale_pose_sha256": "fixture-rest",
        "new_plan_sha256": "fixture-plan",
        "new_independent_dof": 0,
        "new_fixed_root_count": 0,
        "selected": True,
    }
    audit["row_content_sha256"] = self_hash(audit, "row_content_sha256")
    selection_identity = {
        "parent_ordinal": 0,
        "asset_id": asset_id,
        "source_row_sha256": "fixture-source",
        "selection_reasons": ["expanded_rest_normalizer"],
    }
    database = root / "overlay.sqlite3"
    connection = sqlite3.connect(database)
    connection.executescript(
        """
        CREATE TABLE meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);
        CREATE TABLE selection(
          selection_index INTEGER PRIMARY KEY,parent_ordinal INTEGER UNIQUE,
          asset_id TEXT UNIQUE,source_row_sha256 TEXT,reasons_json TEXT,
          new_scale_pose_sha256 TEXT,new_plan_sha256 TEXT,
          new_independent_dof INTEGER,new_fixed_root_count INTEGER,
          audit_json TEXT,row_content_sha256 TEXT);
        CREATE TABLE results(
          parent_ordinal INTEGER PRIMARY KEY,asset_id TEXT UNIQUE,
          record_json TEXT,states_zlib BLOB,state_count INTEGER,
          worker_status TEXT,worker_seconds REAL,completed_at_utc TEXT,
          FOREIGN KEY(parent_ordinal) REFERENCES selection(parent_ordinal));
        """
    )
    connection.executemany(
        "INSERT INTO meta VALUES(?,?)",
        [
            ("schema_version", canonical_text(OVERLAY_DB_SCHEMA)),
            ("run_state", canonical_text("PREPARED")),
        ],
    )
    connection.execute(
        "INSERT INTO selection VALUES(?,?,?,?,?,?,?,?,?,?,?)",
        (
            0,
            0,
            asset_id,
            "fixture-source",
            canonical_text(["expanded_rest_normalizer"]),
            "fixture-rest",
            "fixture-plan",
            0,
            0,
            canonical_text(audit),
            audit["row_content_sha256"],
        ),
    )
    record, states = _fixture_record(asset_id, 0, runtime)
    state_bytes = b"".join(
        (canonical_text(state) + "\n").encode("utf-8") for state in states
    )
    connection.execute(
        "INSERT INTO results VALUES(?,?,?,?,?,?,?,?)",
        (
            0,
            asset_id,
            canonical_text(record),
            sqlite3.Binary(zlib.compress(state_bytes)),
            len(states),
            "completed",
            0.01,
            utc_now(),
        ),
    )
    connection.commit()
    connection.close()
    hasher = CanonicalListHasher()
    hasher.add(selection_identity)
    selected_hash, _ = hasher.finish()
    manifest: dict[str, Any] = {
        "schema_version": RUN_SCHEMA,
        "output": str(root.resolve()),
        "correction_sources": source_bindings,
        "selection": {
            "database": str(database.resolve()),
            "selected_asset_count": 1,
            "corrective_selected_asset_count": 1,
            "reason_counts": {"expanded_rest_normalizer": 1},
            "selected_assets_sha256": selected_hash,
            "include_backend_capacity_errors": False,
            "high_capacity_runtime_attested": False,
        },
    }
    manifest["manifest_content_sha256"] = self_hash(
        manifest, "manifest_content_sha256"
    )
    (root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return database, record, states


def _replace_fixture_record(database: Path, record: Mapping[str, Any]) -> None:
    connection = sqlite3.connect(database)
    connection.execute(
        "UPDATE results SET record_json=? WHERE parent_ordinal=0",
        (canonical_text(record),),
    )
    connection.commit()
    connection.close()


def self_test() -> dict[str, Any]:
    rejected: list[str] = []
    with tempfile.TemporaryDirectory(prefix="pva-v2-acceptance-fixture-") as name:
        root = Path(name)
        database, good_record, _states = _write_fixture(root)
        passing = check_acceptance(
            root,
            expected_assets=1,
            expected_reason_counts={"expanded_rest_normalizer": 1},
        )
        if passing.get("status") != "PASS":
            raise AcceptanceError("passing fixture did not pass")
        mutations = {
            "phase_incomplete": {"rest_state_executed": 0},
            "runtime_mismatch": {
                "runtime_identity": {
                    **dict(good_record["runtime_identity"]),
                    "pybullet_module_sha256": "0" * 64,
                }
            },
            "normalization_mismatch": {
                "normalization_joint_values_sha256": "wrong-rest"
            },
        }
        for label, changes in mutations.items():
            bad = {**good_record, **changes}
            _replace_fixture_record(database, bad)
            try:
                check_acceptance(
                    root,
                    expected_assets=1,
                    expected_reason_counts={"expanded_rest_normalizer": 1},
                )
            except AcceptanceError:
                rejected.append(label)
            else:
                raise AcceptanceError(f"negative fixture was accepted: {label}")
            _replace_fixture_record(database, good_record)
    result: dict[str, Any] = {
        "schema_version": SELF_TEST_SCHEMA,
        "status": "PASS",
        "positive_fixture_passed": True,
        "negative_fixtures_rejected": sorted(rejected),
    }
    result["self_test_content_sha256"] = self_hash(
        result, "self_test_content_sha256"
    )
    return result


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    subparsers = value.add_subparsers(dest="command", required=True)
    check = subparsers.add_parser("check", help="check the formal default-847 overlay")
    check.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    subparsers.add_parser("self-test", help="run positive and negative fixtures")
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        report = self_test() if args.command == "self-test" else check_acceptance(args.output)
    except (AcceptanceError, OSError, sqlite3.Error, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
