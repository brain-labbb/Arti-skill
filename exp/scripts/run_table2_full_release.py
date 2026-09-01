#!/usr/bin/env python3
"""Run Table 2 over an immutable full-release roster.

The legacy Table 2 evaluator remains the sole source of metric semantics.  This
adapter owns only release-roster binding, bounded child execution, checkpointing,
and deterministic aggregation.  It intentionally accepts the roster format
from :mod:`table123_full_release_common`, never a legacy N=800 cohort manifest.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import suppress
import importlib.util
import json
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any, Mapping

import table123_full_release_common as common


SCRIPT_PATH = Path(__file__).resolve()
CORE_PATH = SCRIPT_PATH.with_name("run_table2_urdf_articraft.py")
RUN_SCHEMA_VERSION = "table2_full_release_run_v1"
ARTIFACT_SCHEMA_VERSION = common.ARTIFACT_SCHEMA_VERSION
BLAS_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def _load_core() -> Any:
    existing = sys.modules.get("table2_urdf_articraft_full_release_core")
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(
        "table2_urdf_articraft_full_release_core", CORE_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import Table 2 core: {CORE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


core = _load_core()
METRIC_NAMES = tuple(core.METRIC_NAMES)


def _metadata(row: Mapping[str, Any]) -> dict[str, Any]:
    """Copy stable roster labels onto an evaluator record."""

    fields = (
        "asset_id",
        "source_relative_path",
        "primary_urdf_relative_path",
        "primary_urdf_sha256",
        "package_binding_sha256",
        "raw_category",
        "category",
        "release_tier",
        "tier",
        "object_release_id",
        "model_id",
        "source_repository",
    )
    return {field: row[field] for field in fields if field in row}


def _failure_record(row: Mapping[str, Any], reason: str, *, status: str = "error") -> dict[str, Any]:
    package = Path(str(row.get("source_path") or row.get("package_root") or ""))
    asset_id = str(row.get("asset_id", "unknown"))
    primary = str(row.get("primary_urdf_relative_path", "model.urdf"))
    record = core.failed_record(asset_id, package, reason, primary_urdf_relative_path=primary)
    record["status"] = status
    record.update(_metadata(row))
    record["result_origin"] = "parent_synthesized"
    return record


def failure_record(row: Mapping[str, Any], reason: str, *, status: str = "error") -> dict[str, Any]:
    """Return a denominator-preserving error/timeout record for one roster row."""

    return _failure_record(row, reason, status=status)


def _core_primary_relative_path(
    row: Mapping[str, Any], package: Path, declared: str
) -> str:
    """Translate the roster's frozen primary path to the core's package scope.

    Full-release roster rows retain ``primary_urdf_relative_path`` relative to
    the dataset source root for portable provenance.  The legacy Table 2 core
    deliberately accepts only a path relative to its package argument.  The
    absolute ``primary_urdf_path`` is authoritative, so derive that package-
    relative spelling and fail closed if it escapes the package.
    """

    absolute_value = row.get("primary_urdf_path")
    if not isinstance(absolute_value, str) or not absolute_value:
        return declared
    primary = Path(absolute_value)
    if not primary.is_absolute():
        primary = package / primary
    package_root = package.resolve(strict=False)
    try:
        relative = primary.relative_to(package_root)
    except ValueError as error:
        raise ValueError("frozen primary URDF escapes source package") from error
    return relative.as_posix()


def audit_row(row: Mapping[str, Any], run_standard_parser: bool = True) -> dict[str, Any]:
    """Audit one roster row using the existing Table 2 metric core."""

    if not isinstance(row, Mapping) or not isinstance(row.get("asset_id"), str):
        raise ValueError("roster row requires a string asset_id")
    package_value = row.get("source_path") or row.get("package_root")
    if not isinstance(package_value, str) or not package_value:
        raise ValueError(f"roster row has no source package: {row['asset_id']}")
    package = Path(package_value)
    declared_primary = str(row.get("primary_urdf_relative_path", "model.urdf"))
    primary = declared_primary
    expected_hash = row.get("primary_urdf_sha256")
    observed_hash: str | None = None
    try:
        # Hash the primary input on both sides of the core audit.  This closes
        # the evaluation-time TOCTOU window while keeping the roster's frozen
        # hash authoritative when a child fails or times out.
        primary = _core_primary_relative_path(row, package, declared_primary)
        primary_path = core.primary_urdf_path(package, primary)
        observed_hash = common.sha256_file(primary_path)
        if expected_hash is not None and observed_hash != expected_hash:
            raise ValueError("primary URDF hash drift before audit")
        record = core.audit_asset_package(
            package,
            run_standard_parser=run_standard_parser,
            asset_id=str(row["asset_id"]),
            primary_urdf_relative_path=primary,
        )
        after_hash = common.sha256_file(primary_path)
        if after_hash != observed_hash:
            raise ValueError("primary URDF changed during evaluation")
    except BaseException as error:  # noqa: BLE001 - a failed asset stays in the denominator
        failure = _failure_record(
            row,
            f"audit_exception: {type(error).__name__}: {error}",
        )
        failure["primary_urdf_sha256"] = expected_hash or observed_hash
        return failure
    record.update(_metadata(row))
    record["primary_urdf_sha256"] = expected_hash or observed_hash
    record["result_origin"] = "child_audit"
    return record


def _category_field(roster_rows: list[Mapping[str, Any]], records: list[Mapping[str, Any]]) -> str | None:
    for field in ("raw_category", "category"):
        if all(isinstance(row.get(field), str) and row.get(field) for row in roster_rows):
            if all(isinstance(record.get(field), str) and record.get(field) for record in records):
                return field
    return None


def aggregate_full_release(records: list[dict[str, Any]], roster: Mapping[str, Any]) -> dict[str, Any]:
    """Aggregate all records against the roster's dynamic N/J denominator."""

    if not isinstance(roster, Mapping) or roster.get("schema_version") != common.SCHEMA_VERSION:
        raise ValueError("full-release roster required for aggregation")
    rows = roster.get("rows")
    if not isinstance(rows, list):
        raise ValueError("full-release roster rows are missing")
    expected_ids = [str(row["asset_id"]) for row in rows]
    expected_id_set = set(expected_ids)
    if len(expected_id_set) != len(expected_ids):
        raise ValueError("full-release roster asset IDs are not unique")
    by_id: dict[str, dict[str, Any]] = {}
    for record in records:
        asset_id = record.get("asset_id")
        if not isinstance(asset_id, str) or asset_id in by_id or asset_id not in expected_id_set:
            raise ValueError(f"record identity is not unique and roster-bound: {asset_id!r}")
        by_id[asset_id] = record
    for row in rows:
        asset_id = str(row["asset_id"])
        record = by_id.get(asset_id)
        if record is None:
            continue
        expected_hash = row.get("primary_urdf_sha256")
        if expected_hash is not None and record.get("primary_urdf_sha256") != expected_hash:
            raise ValueError(f"record source hash mismatch for {asset_id}")
    ordered = [by_id[asset_id] for asset_id in expected_ids if asset_id in by_id]
    field = _category_field(rows, ordered) if ordered else None
    summary = core.aggregate_records(ordered, expected_n=len(rows), category_field=field)
    summary.update(
        {
            "schema_version": RUN_SCHEMA_VERSION,
            "dataset": roster.get("dataset"),
            "n_eval": len(rows),
            "j_eval": int(roster.get("J_eval", 0)),
            "roster_sha256": roster.get("roster_sha256"),
            "roster_manifest_content_sha256": roster.get("manifest_content_sha256"),
            "denominator_policy": "all full-release roster rows, including child errors and timeouts",
        }
    )
    return summary


def _child_entry(row: dict[str, Any], parser: bool, connection: Any) -> None:
    """Execute one audit in an owned process; used for hard timeouts."""

    try:
        os.environ.update(BLAS_ENVIRONMENT)
        record = audit_row(row, parser)
        record["evaluation_environment"] = {
            name: os.environ.get(name) for name in BLAS_ENVIRONMENT
        }
        connection.send(record)
    except BaseException as error:  # noqa: BLE001
        connection.send(_failure_record(row, f"child_exception: {type(error).__name__}: {error}"))
    finally:
        connection.close()


def _run_rows(
    rows: list[dict[str, Any]],
    *,
    workers: int,
    timeout_seconds: float,
    run_standard_parser: bool,
    on_record: Any,
) -> None:
    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    context = multiprocessing.get_context("fork" if "fork" in multiprocessing.get_all_start_methods() else "spawn")
    pending = list(enumerate(rows))
    active: dict[int, tuple[int, Any, Any, float]] = {}
    completed = 0
    try:
        while pending or active:
            while pending and len(active) < workers:
                ordinal, row = pending.pop(0)
                receive, send = context.Pipe(duplex=False)
                process = context.Process(target=_child_entry, args=(row, run_standard_parser, send))
                process.start()
                send.close()
                active[process.pid] = (ordinal, process, receive, time.monotonic())
            for pid, state in list(active.items()):
                ordinal, process, receive, started = state
                # ``multiprocessing.Connection.send`` writes one pickled
                # object to a bounded pipe.  Some full-release audits carry a
                # large resource-reference/mesh diagnostic payload; if the
                # parent waits for process exit before reading, the child can
                # block in ``send`` forever and be misclassified as a timeout.
                # Drain a completed result while the child is still alive,
                # then give it a short grace period to exit cleanly.
                if receive.poll():
                    try:
                        record = receive.recv()
                    except (EOFError, OSError) as error:
                        record = _failure_record(
                            rows[ordinal],
                            f"child_result_invalid: {type(error).__name__}: {error}",
                        )
                    if process.is_alive():
                        process.join(timeout=0.2)
                    if process.is_alive():
                        with suppress(Exception):
                            process.terminate()
                        process.join(timeout=0.2)
                    record["ordinal"] = ordinal
                    record.setdefault("evaluation_environment", dict(BLAS_ENVIRONMENT))
                    on_record(record)
                    receive.close()
                    del active[pid]
                    completed += 1
                    continue
                timed_out = time.monotonic() - started >= timeout_seconds and process.is_alive()
                if process.is_alive() and not timed_out:
                    continue
                if timed_out:
                    process.terminate()
                    process.join(timeout=0.2)
                    if process.is_alive():
                        with suppress(Exception):
                            process.kill()
                        process.join(timeout=0.2)
                    record = _failure_record(rows[ordinal], f"asset_timeout_after_{timeout_seconds:g}_seconds", status="timeout")
                else:
                    process.join(timeout=0.2)
                    if receive.poll():
                        try:
                            record = receive.recv()
                        except (EOFError, OSError) as error:
                            record = _failure_record(rows[ordinal], f"child_result_invalid: {type(error).__name__}: {error}")
                    else:
                        record = _failure_record(rows[ordinal], f"child_exit_{process.exitcode}")
                record["ordinal"] = ordinal
                record.setdefault("evaluation_environment", dict(BLAS_ENVIRONMENT))
                on_record(record)
                receive.close()
                del active[pid]
                completed += 1
            if active:
                time.sleep(0.005)
    finally:
        for _ordinal, process, receive, _started in active.values():
            with suppress(Exception):
                process.terminate()
            with suppress(Exception):
                process.join(timeout=0.2)
            receive.close()


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    payload = "".join(json.dumps(record, sort_keys=True, ensure_ascii=True) + "\n" for record in records)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(path)


def _run_manifest(roster: Mapping[str, Any], *, workers: int, timeout_seconds: float, parser: bool) -> dict[str, Any]:
    manifest = {
        "schema_version": RUN_SCHEMA_VERSION,
        "dataset": roster.get("dataset"),
        "roster_manifest_content_sha256": roster.get("manifest_content_sha256"),
        "roster_sha256": roster.get("roster_sha256"),
        "N_eval": roster.get("N_eval"),
        "J_eval": roster.get("J_eval"),
        "workers": workers,
        "timeout_seconds": timeout_seconds,
        "run_standard_parser": parser,
        "blas_environment": dict(BLAS_ENVIRONMENT),
        "denominator_policy": "all full-release roster rows, including child errors and timeouts",
    }
    manifest["manifest_content_sha256"] = common.canonical_sha256(manifest)
    return manifest


def _write_artifact_manifest(output: Path) -> None:
    names = ["manifest.json", "asset_records.jsonl", "summary.json", "checkpoint.json"]
    artifacts = [
        {"path": name, "size": (output / name).stat().st_size, "sha256": common.sha256_file(output / name)}
        for name in names
    ]
    payload: dict[str, Any] = {"schema_version": ARTIFACT_SCHEMA_VERSION, "artifacts": artifacts}
    payload["artifact_manifest_content_sha256"] = common.canonical_sha256(payload)
    common._atomic_write_json(output / "artifact_manifest.json", payload)


def _verify_checkpoint(path: Path, run_manifest: Mapping[str, Any], roster: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate a resumable checkpoint before trusting its records."""

    try:
        checkpoint = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"checkpoint is invalid: {error}") from error
    if not isinstance(checkpoint, Mapping):
        raise ValueError("checkpoint must be an object")
    declared = checkpoint.get("checkpoint_content_sha256")
    unsigned = dict(checkpoint)
    unsigned.pop("checkpoint_content_sha256", None)
    if declared != common.canonical_sha256(unsigned):
        raise ValueError("checkpoint self-hash mismatch")
    if checkpoint.get("manifest_content_sha256") != run_manifest.get("manifest_content_sha256"):
        raise ValueError("checkpoint manifest binding mismatch")
    if checkpoint.get("roster_manifest_content_sha256") != roster.get("manifest_content_sha256"):
        raise ValueError("checkpoint roster binding mismatch")
    if checkpoint.get("n_eval") != roster.get("N_eval"):
        raise ValueError("checkpoint denominator mismatch")
    return checkpoint


def run_full_release(
    roster_path: Path,
    output: Path,
    workers: int,
    timeout_seconds: float,
    *,
    run_standard_parser: bool = True,
    resume: bool = False,
) -> Path:
    """Evaluate every row in a full-release roster and write a resumable receipt."""

    try:
        # Rebind every source package before starting workers.  This keeps
        # package additions/removals/content drift out of the published
        # denominator while the common helper skips the intentionally deferred
        # PhysX shared-container binding.
        roster = common.load_roster(Path(roster_path), verify_sources="auto")
    except Exception as error:  # noqa: BLE001
        raise ValueError(f"full-release roster required (legacy N=800 manifests are rejected): {error}") from error
    rows = [dict(row) for row in roster["rows"]]
    if not rows:
        raise ValueError("full-release roster cannot be empty")
    output = Path(output).resolve()
    if resume:
        if not output.is_dir():
            raise ValueError("resume output does not exist")
        run_manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        if run_manifest.get("schema_version") != RUN_SCHEMA_VERSION:
            raise ValueError("resume output is not a full-release Table 2 run")
        declared_manifest_hash = run_manifest.get("manifest_content_sha256")
        unsigned_manifest = dict(run_manifest)
        unsigned_manifest.pop("manifest_content_sha256", None)
        if declared_manifest_hash != common.canonical_sha256(unsigned_manifest):
            raise ValueError("run manifest self-hash mismatch")
        expected = _run_manifest(roster, workers=workers, timeout_seconds=timeout_seconds, parser=run_standard_parser)
        for key in ("roster_manifest_content_sha256", "roster_sha256", "N_eval", "J_eval", "workers", "timeout_seconds", "run_standard_parser"):
            if run_manifest.get(key) != expected.get(key):
                raise ValueError(f"resume binding mismatch: {key}")
        checkpoint_path = output / "checkpoint.json"
        if not checkpoint_path.exists():
            raise ValueError("resume checkpoint is missing")
        checkpoint = _verify_checkpoint(checkpoint_path, run_manifest, roster)
        artifact_path = output / "artifact_manifest.json"
        if artifact_path.exists():
            common.verify_artifacts(artifact_path)
        elif checkpoint.get("state") == "complete":
            raise ValueError("resume artifact manifest is missing")
        records_path = output / "asset_records.jsonl"
        declared_records_hash = checkpoint.get("asset_records_sha256")
        if declared_records_hash is not None:
            if not records_path.exists() or common.sha256_file(records_path) != declared_records_hash:
                raise ValueError("checkpoint asset records hash mismatch")
        records = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines() if line.strip()] if records_path.exists() else []
    else:
        if output.exists() and any(output.iterdir()):
            raise ValueError(f"output already exists and is not empty: {output}")
        output.mkdir(parents=True, exist_ok=True)
        run_manifest = _run_manifest(roster, workers=workers, timeout_seconds=timeout_seconds, parser=run_standard_parser)
        common._atomic_write_json(output / "manifest.json", run_manifest)
        records = []
    by_id = {record.get("asset_id"): record for record in records}
    expected_ids = [row["asset_id"] for row in rows]
    if any(asset_id not in expected_ids for asset_id in by_id) or len(by_id) != len(records):
        raise ValueError("existing records contain duplicate or foreign asset IDs")
    # A pre-batch runner version could leave ordinals relative to a resumed
    # pending slice.  Rebind them to the immutable roster before checkpointing.
    for ordinal, row in enumerate(rows):
        record = by_id.get(row["asset_id"])
        if record is not None:
            record["ordinal"] = row.get("ordinal", ordinal)
    pending = [row for row in rows if row["asset_id"] not in by_id]
    common.write_checkpoint(output / "checkpoint.json", {
        "state": "running" if pending else "complete",
        "manifest_content_sha256": run_manifest["manifest_content_sha256"],
        "roster_manifest_content_sha256": roster["manifest_content_sha256"],
        "completed_ordinals": sorted(int(by_id[asset_id].get("ordinal", expected_ids.index(asset_id))) for asset_id in by_id),
        "n_eval": len(rows),
        **({"asset_records_sha256": common.sha256_file(output / "asset_records.jsonl")} if (output / "asset_records.jsonl").exists() else {}),
    })

    pending_since_flush = 0

    def persist_records() -> None:
        """Persist the deterministic roster order and its resumable hash."""

        ordered = [by_id[asset_id] for asset_id in expected_ids if asset_id in by_id]
        _write_jsonl(output / "asset_records.jsonl", ordered)
        common.write_checkpoint(output / "checkpoint.json", {
            "state": "running",
            "manifest_content_sha256": run_manifest["manifest_content_sha256"],
            "roster_manifest_content_sha256": roster["manifest_content_sha256"],
            "completed_ordinals": [int(item.get("ordinal", 0)) for item in ordered],
            "n_eval": len(rows),
            "asset_records_sha256": common.sha256_file(output / "asset_records.jsonl"),
        })

    def save(record: dict[str, Any]) -> None:
        nonlocal pending_since_flush
        by_id[record["asset_id"]] = record
        pending_since_flush += 1
        # Rewriting a large deterministic JSONL after every asset makes a
        # full release quadratic on shared storage.  A bounded batch keeps
        # crash recovery loss below one batch while preserving the exact
        # ordered artifact at every flush and at final completion.
        if pending_since_flush >= 32:
            persist_records()
            pending_since_flush = 0

    _run_rows(
        pending,
        workers=workers,
        timeout_seconds=timeout_seconds,
        run_standard_parser=run_standard_parser,
        on_record=save,
    )
    if pending_since_flush:
        persist_records()
    ordered = [by_id[asset_id] for asset_id in expected_ids]
    summary = aggregate_full_release(ordered, roster)
    common._atomic_write_json(output / "summary.json", summary)
    common.write_checkpoint(output / "checkpoint.json", {
        "state": "complete",
        "manifest_content_sha256": run_manifest["manifest_content_sha256"],
        "roster_manifest_content_sha256": roster["manifest_content_sha256"],
        "completed_ordinals": list(range(len(rows))),
        "n_eval": len(rows),
        "asset_records_sha256": common.sha256_file(output / "asset_records.jsonl"),
    })
    _write_artifact_manifest(output)
    common.verify_artifacts(output)
    return output


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roster", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--no-standard-parser", action="store_true")
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        output = run_full_release(
            args.roster,
            args.output,
            args.workers,
            args.timeout_seconds,
            run_standard_parser=not args.no_standard_parser,
            resume=args.resume,
        )
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "completed", "output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
