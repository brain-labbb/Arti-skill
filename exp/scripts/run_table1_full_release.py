#!/usr/bin/env python3
"""Evaluate Table 1 metrics over an immutable full-release roster."""

from __future__ import annotations

import argparse
import importlib.util
import json
import multiprocessing
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import table123_full_release_common as common


SCRIPT_PATH = Path(__file__).resolve()
CORE_PATH = SCRIPT_PATH.with_name("run_table1_artiverse.py")
RUN_SCHEMA_VERSION = "table1_full_release_run_v1"
BLAS_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def _load_core() -> Any:
    existing = sys.modules.get("table1_artiverse_full_release_core")
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location("table1_artiverse_full_release_core", CORE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load Table 1 core: {CORE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


core = _load_core()


def _metadata(row: Mapping[str, Any]) -> dict[str, Any]:
    fields = (
        "asset_id", "ordinal", "source_relative_path", "primary_urdf_relative_path",
        "raw_category", "category", "release_tier", "tier", "object_release_id",
        "model_id", "source_repository", "primary_urdf_sha256", "package_binding_sha256",
    )
    metadata = {field: row[field] for field in fields if field in row}
    if "raw_category" not in metadata and "category" in metadata:
        metadata["raw_category"] = metadata["category"]
    return metadata


def failure_record(row: Mapping[str, Any], reason: str, *, status: str = "error") -> dict[str, Any]:
    record = {
        **_metadata(row),
        "asset_id": str(row["asset_id"]),
        "status": status,
        "parse_success": False,
        "link_count": None,
        "joint_count": None,
        "joint_type_counts": None,
        "non_fixed_joint_count": None,
        "valid_tree": False,
        "topology_hash": None,
        "fingerprint_complete": False,
        "package_fingerprint": None,
        "referenced_resource_count": None,
        "missing_resources": [],
        "error": reason,
    }
    record["result_origin"] = "parent_synthesized"
    return record


def audit_row(row: Mapping[str, Any], run_standard_parser: bool = True) -> dict[str, Any]:
    """Evaluate one roster row with shared analyze/fingerprint semantics."""

    del run_standard_parser  # retained for adapter API parity; Table 1 core is static-only
    if not isinstance(row, Mapping) or not isinstance(row.get("asset_id"), str):
        raise ValueError("roster row requires a string asset_id")
    record = failure_record(row, "evaluation not started")
    path_value = row.get("primary_urdf_path") or row.get("urdf_path")
    if not isinstance(path_value, str) or not path_value:
        record["error"] = "primary URDF path is missing"
        return record
    path = Path(path_value)
    try:
        if path.is_symlink() or not path.is_file():
            raise OSError("primary URDF is missing or is a symlink")
        expected_hash = row.get("primary_urdf_sha256")
        if expected_hash and common.sha256_file(path) != expected_hash:
            raise ValueError("primary URDF hash drift")
        record.update(core.analyze_urdf(path))
        record["parse_success"] = True
        package_root_value = row.get("source_path") or row.get("package_root")
        package_root = Path(str(package_root_value)) if package_root_value else path.parent
        fingerprint = core.fingerprint_package(path, package_root=package_root)
        record.update({
            "fingerprint_complete": bool(fingerprint.get("complete")),
            "package_fingerprint": fingerprint.get("fingerprint"),
            "referenced_resource_count": fingerprint.get("resource_count"),
            "missing_resources": fingerprint.get("missing_resources", []),
        })
        record["status"] = "EVALUATED" if fingerprint.get("complete") else "EVALUATED_FINGERPRINT_INCOMPLETE"
        if not fingerprint.get("complete"):
            record["error"] = "one or more referenced package resources are unavailable"
        else:
            record["error"] = None
        if expected_hash and common.sha256_file(path) != expected_hash:
            return failure_record(row, "primary URDF changed during evaluation")
    except Exception as error:  # noqa: BLE001 - every asset remains in denominator
        record = failure_record(row, f"{type(error).__name__}: {error}")
    record["result_origin"] = "child_audit"
    return record


def aggregate_full_release(records: list[dict[str, Any]], roster: Mapping[str, Any]) -> dict[str, Any]:
    if roster.get("schema_version") != common.SCHEMA_VERSION:
        raise ValueError("full-release roster required for aggregation")
    rows = roster.get("rows")
    if not isinstance(rows, list) or not rows:
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
    if len(by_id) != len(expected_ids):
        raise ValueError("record count does not match full-release denominator")
    ordered = [by_id[asset_id] for asset_id in expected_ids]
    summary = core.aggregate_records(
        ordered,
        release_asset_count=len(rows),
        release_category_count=len({str(row.get("raw_category", row.get("category", ""))) for row in rows}),
    )
    summary.update({
        "schema_version": RUN_SCHEMA_VERSION,
        "dataset": roster.get("dataset"),
        "n_eval": len(rows),
        "j_eval": int(roster.get("J_eval", 0)),
        "roster_sha256": roster.get("roster_sha256"),
        "roster_manifest_content_sha256": roster.get("manifest_content_sha256"),
        "denominator_policy": "all full-release roster rows, including failures and timeouts",
        "asset_failure_count": sum(
            str(record.get("status")) != "EVALUATED" for record in ordered
        ),
    })
    return summary


def _child(row: dict[str, Any], conn: Any, run_standard_parser: bool) -> None:
    try:
        os.environ.update(BLAS_ENVIRONMENT)
        record = audit_row(row, run_standard_parser)
        record["evaluation_environment"] = {key: os.environ.get(key) for key in BLAS_ENVIRONMENT}
        conn.send(record)
    except BaseException as error:  # noqa: BLE001
        conn.send(failure_record(row, f"child_exception: {type(error).__name__}: {error}"))
    finally:
        conn.close()


def _evaluate_rows(rows: list[dict[str, Any]], *, workers: int, timeout_seconds: float, run_standard_parser: bool, save: Any) -> None:
    if workers <= 0 or timeout_seconds <= 0:
        raise ValueError("workers and timeout_seconds must be positive")
    context = multiprocessing.get_context("fork" if "fork" in multiprocessing.get_all_start_methods() else "spawn")
    pending = list(enumerate(rows))
    active: dict[int, tuple[int, Any, Any, float]] = {}
    while pending or active:
        while pending and len(active) < workers:
            ordinal, row = pending.pop(0)
            receive, send = context.Pipe(duplex=False)
            process = context.Process(target=_child, args=(row, send, run_standard_parser))
            process.start(); send.close()
            active[process.pid] = (ordinal, process, receive, time.monotonic())
        for pid, state in list(active.items()):
            ordinal, process, receive, started = state
            # A full-release Table 1 record can contain a sizeable recursive
            # resource fingerprint.  Drain the pipe as soon as the child has
            # sent its result; waiting for process exit first can deadlock a
            # child in ``Connection.send`` once the pipe buffer fills.
            if receive.poll():
                try:
                    record = receive.recv()
                except (EOFError, OSError) as error:
                    record = failure_record(rows[ordinal], f"child_result_invalid: {error}")
                if process.is_alive():
                    process.join(0.2)
                if process.is_alive():
                    process.terminate(); process.join(0.2)
                record.update(_metadata(rows[ordinal])); record["ordinal"] = ordinal
                save(record)
                receive.close(); del active[pid]
                continue
            if process.is_alive() and time.monotonic() - started < timeout_seconds:
                continue
            if process.is_alive():
                process.terminate(); process.join(0.2)
                record = failure_record(rows[ordinal], f"asset_timeout_after_{timeout_seconds:g}_seconds", status="timeout")
            else:
                process.join(0.2)
                try:
                    record = receive.recv() if receive.poll() else failure_record(rows[ordinal], f"child_exit_{process.exitcode}")
                except (EOFError, OSError) as error:
                    record = failure_record(rows[ordinal], f"child_result_invalid: {error}")
            record.update(_metadata(rows[ordinal])); record["ordinal"] = ordinal
            save(record)
            receive.close(); del active[pid]
        if active:
            time.sleep(0.005)


def _run_manifest(roster: Mapping[str, Any], workers: int, timeout_seconds: float, run_standard_parser: bool) -> dict[str, Any]:
    manifest = {
        "schema_version": RUN_SCHEMA_VERSION,
        "dataset": roster.get("dataset"),
        "roster_manifest_content_sha256": roster.get("manifest_content_sha256"),
        "roster_sha256": roster.get("roster_sha256"),
        "N_eval": roster.get("N_eval"), "J_eval": roster.get("J_eval"),
        "workers": workers, "timeout_seconds": timeout_seconds,
        "run_standard_parser": run_standard_parser,
        "denominator_policy": "all full-release roster rows, including failures and timeouts",
    }
    manifest["manifest_content_sha256"] = common.canonical_sha256(manifest)
    return manifest


def _write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    common._atomic_write_bytes(path, b"".join(common._canonical_bytes(row) + b"\n" for row in records))


def _write_artifact_manifest(output: Path) -> None:
    names = [
        "manifest.json",
        "roster_manifest.json",
        "full_release_roster.jsonl",
        "asset_records.jsonl",
        "summary.json",
        "checkpoint.json",
    ]
    artifact = {"schema_version": common.ARTIFACT_SCHEMA_VERSION, "artifacts": [
        {"path": name, "size": (output / name).stat().st_size, "sha256": common.sha256_file(output / name)} for name in names
    ]}
    artifact["artifact_manifest_content_sha256"] = common.canonical_sha256(artifact)
    common._atomic_write_json(output / "artifact_manifest.json", artifact)


def _copy_roster_artifacts(roster_path: Path, output: Path) -> None:
    source_manifest = Path(roster_path).read_bytes()
    source_rows = Path(roster_path).with_name("full_release_roster.jsonl").read_bytes()
    for name, payload in (("roster_manifest.json", source_manifest), ("full_release_roster.jsonl", source_rows)):
        target = output / name
        if target.exists() and target.read_bytes() != payload:
            raise ValueError(f"existing {name} differs from frozen roster")
        if not target.exists():
            common._atomic_write_bytes(target, payload)


def _verify_checkpoint(path: Path, manifest: Mapping[str, Any], roster: Mapping[str, Any]) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"checkpoint is invalid: {error}") from error
    unsigned = dict(value); declared = unsigned.pop("checkpoint_content_sha256", None)
    if declared != common.canonical_sha256(unsigned):
        raise ValueError("checkpoint self-hash mismatch")
    if value.get("manifest_content_sha256") != manifest.get("manifest_content_sha256") or value.get("roster_manifest_content_sha256") != roster.get("manifest_content_sha256"):
        raise ValueError("checkpoint binding mismatch")
    if value.get("n_eval") != roster.get("N_eval"):
        raise ValueError("checkpoint denominator mismatch")
    return value


def run_full_release(
    roster_path: Path,
    output: Path,
    workers: int,
    timeout_seconds: float,
    *,
    run_standard_parser: bool = True,
    resume: bool = False,
) -> Path:
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
    expected_manifest = _run_manifest(roster, workers, timeout_seconds, run_standard_parser)
    if resume:
        if not output.is_dir():
            raise ValueError("resume output does not exist")
        run_manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        unsigned = dict(run_manifest); declared = unsigned.pop("manifest_content_sha256", None)
        if declared != common.canonical_sha256(unsigned):
            raise ValueError("run manifest self-hash mismatch")
        for key in ("roster_manifest_content_sha256", "roster_sha256", "N_eval", "J_eval", "workers", "timeout_seconds", "run_standard_parser"):
            if run_manifest.get(key) != expected_manifest.get(key):
                raise ValueError(f"resume binding mismatch: {key}")
        checkpoint = _verify_checkpoint(output / "checkpoint.json", run_manifest, roster)
        if (output / "artifact_manifest.json").exists():
            common.verify_artifacts(output)
        records_path = output / "asset_records.jsonl"
        if checkpoint.get("asset_records_sha256") and common.sha256_file(records_path) != checkpoint["asset_records_sha256"]:
            raise ValueError("checkpoint asset records hash mismatch")
        records = [json.loads(line) for line in records_path.read_text(encoding="utf-8").splitlines() if line.strip()] if records_path.exists() else []
    else:
        if output.exists() and any(output.iterdir()):
            raise ValueError(f"output already exists and is not empty: {output}")
        output.mkdir(parents=True, exist_ok=True)
        run_manifest = expected_manifest
        common._atomic_write_json(output / "manifest.json", run_manifest)
        records = []
    _copy_roster_artifacts(Path(roster_path), output)
    expected_ids = [str(row["asset_id"]) for row in rows]
    by_id = {str(record.get("asset_id")): record for record in records}
    if len(by_id) != len(records) or any(asset_id not in expected_ids for asset_id in by_id):
        raise ValueError("existing records contain duplicate or foreign asset IDs")
    # Older interrupted runs could persist a local ordinal from the pending
    # slice after resume.  Asset identity and roster order are authoritative;
    # normalize the field before any checkpoint or aggregation is written.
    for ordinal, row in enumerate(rows):
        record = by_id.get(str(row["asset_id"]))
        if record is not None:
            record["ordinal"] = row.get("ordinal", ordinal)

    def save(record: dict[str, Any]) -> None:
        by_id[str(record["asset_id"])] = record
        ordered = [by_id[asset_id] for asset_id in expected_ids if asset_id in by_id]
        _write_jsonl(output / "asset_records.jsonl", ordered)
        common.write_checkpoint(output / "checkpoint.json", {
            "state": "running", "manifest_content_sha256": run_manifest["manifest_content_sha256"],
            "roster_manifest_content_sha256": roster["manifest_content_sha256"],
            "completed_ordinals": [int(item.get("ordinal", 0)) for item in ordered], "n_eval": len(rows),
            "asset_records_sha256": common.sha256_file(output / "asset_records.jsonl"),
        })

    pending = [row for row in rows if str(row["asset_id"]) not in by_id]
    common.write_checkpoint(output / "checkpoint.json", {
        "state": "running" if pending else "complete", "manifest_content_sha256": run_manifest["manifest_content_sha256"],
        "roster_manifest_content_sha256": roster["manifest_content_sha256"], "completed_ordinals": sorted(int(by_id[key].get("ordinal", expected_ids.index(key))) for key in by_id), "n_eval": len(rows),
        **({"asset_records_sha256": common.sha256_file(output / "asset_records.jsonl")} if (output / "asset_records.jsonl").exists() else {}),
    })
    _evaluate_rows(
        pending,
        workers=workers,
        timeout_seconds=timeout_seconds,
        run_standard_parser=run_standard_parser,
        save=save,
    )
    ordered = [by_id[asset_id] for asset_id in expected_ids]
    _write_jsonl(output / "asset_records.jsonl", ordered)
    common._atomic_write_json(output / "summary.json", aggregate_full_release(ordered, roster))
    common.write_checkpoint(output / "checkpoint.json", {
        "state": "complete", "manifest_content_sha256": run_manifest["manifest_content_sha256"],
        "roster_manifest_content_sha256": roster["manifest_content_sha256"], "completed_ordinals": list(range(len(rows))), "n_eval": len(rows),
        "asset_records_sha256": common.sha256_file(output / "asset_records.jsonl"),
    })
    _write_artifact_manifest(output)
    common.verify_artifacts(output)
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--roster", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--no-standard-parser", action="store_true")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = run_full_release(
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
    print(json.dumps({"status": "completed", "output": str(result)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
