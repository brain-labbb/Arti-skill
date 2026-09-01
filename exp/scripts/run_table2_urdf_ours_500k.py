#!/usr/bin/env python3
"""Evaluate Table 2 (URDF validity) on the frozen Ours-500K cohort (Brain).

The cohort is the frozen Table 1 Ours-500K full acquired roster
(``exp/runtime/table1_ours_500k``): archive-bound manifest + per-asset
``model.urdf`` hashes, N_release = N_eval = 500.  All nine Table 2 metrics
are computed by the shared Table 2 audit core (``run_table2_urdf_articraft.py``:
``audit_asset_package`` with urdfpy 0.0.22 standard-parser load, resource
closure validation, finite-field/tree/joint-spec audits, collision coverage,
inertial coverage and inertia validity), executed one asset per fresh
interpreter with owned-process-group timeouts.

Unlike PhysX-Mobility, the Ours-500K release layout is already a
self-contained package per asset (``<category>/<seed_N>/model.urdf`` plus an
``assets/`` resource closure), so no staging copy is needed; the audit reads
the release packages in place and the package content binding is re-verified
before and after each audit.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
from typing import Any
import uuid


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = (
    REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
)
DEFAULT_ARCHIVE = REPO / "exp/Brain/arti_cabinet_drawer_geometry_500_20260813.zip"
DEFAULT_ARCHIVE_SHA256_FILE = (
    REPO / "exp/Brain/arti_cabinet_drawer_geometry_500_20260813.zip.sha256"
)
DEFAULT_TABLE1_RECEIPT = REPO / "exp/runtime/table1_ours_500k"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
TABLE2_EVALUATOR_PATH = REPO / "exp/scripts/run_table2_urdf_articraft.py"
TABLE1_RUNNER_PATH = REPO / "exp/scripts/run_table1_ours_500k.py"
SCRIPT_PATH = Path(__file__).resolve()
ASSET_TIMEOUT_SECONDS = 300.0
CHILD_GRACE_SECONDS = 0.2
RUNTIME_BINDING_FATAL_EXIT_CODE = 86
EXPECTED_TABLE1_MANIFEST_SHA256 = (
    "bc3eb334b1fc1c57378e50e7c2fab5d765a7599db8b3e82bc1d91536570b7c06"
)
EXPECTED_ROSTER_SHA256 = (
    "ed70ebeb97f9ad8a655288e2afce96b0c3a8e26f50653e50dbbdc00238cfea3b"
)
EXPECTED_N = 500
DATASET_LABEL = "Ours-500K"
OURS_METADATA_FIELDS = (
    "raw_category",
    "seed_name",
    "asset_root",
    "primary_urdf",
    "table1_selection_rank",
)


class FatalRuntimeBindingError(RuntimeError):
    """The process no longer matches the frozen evaluator provenance."""


def _load_module(path: Path, name: str):
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load required module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


TABLE2 = _load_module(TABLE2_EVALUATOR_PATH, "table2_urdf_audit_core_shared")
TABLE1O = _load_module(TABLE1_RUNNER_PATH, "table2_ours_table1_cohort_shared")
SHARED = TABLE1O.SHARED


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def evaluator_binding() -> dict[str, str]:
    return {
        "evaluator_path": str(TABLE2_EVALUATOR_PATH.resolve()),
        "evaluator_sha256": TABLE2.sha256_file(TABLE2_EVALUATOR_PATH.resolve()),
        "runner_path": str(SCRIPT_PATH),
        "runner_sha256": TABLE2.sha256_file(SCRIPT_PATH),
    }


def environment_binding() -> dict[str, Any]:
    import importlib.metadata

    return {
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "executable": sys.executable,
        "dependencies": {
            name: (
                importlib.metadata.version(name)
                if importlib.util.find_spec(name) is not None
                else None
            )
            for name in TABLE2.EVALUATION_DEPENDENCIES
        },
    }


def _atomic_write_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def load_table1_cohort(receipt_dir: Path) -> dict[str, Any]:
    """Load and validate the frozen Table 1 Ours-500K receipt set."""
    receipt_dir = receipt_dir.resolve(strict=True)
    manifest_path = receipt_dir / "manifest.json"
    manifest_sha = TABLE2.sha256_file(manifest_path)
    if manifest_sha != EXPECTED_TABLE1_MANIFEST_SHA256:
        raise ValueError(
            f"Table 1 manifest hash mismatch: {manifest_sha} != {EXPECTED_TABLE1_MANIFEST_SHA256}"
        )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != "table1_ours_500k_manifest_v1":
        raise ValueError(f"unexpected Table 1 manifest schema: {manifest.get('schema_version')!r}")
    if manifest.get("roster_sha256") != EXPECTED_ROSTER_SHA256:
        raise ValueError("Table 1 roster hash mismatch")
    if manifest.get("N_release") != EXPECTED_N or manifest.get("N_eval") != EXPECTED_N:
        raise ValueError("Table 1 cohort size mismatch")
    rows = manifest.get("assets")
    if not isinstance(rows, list) or len(rows) != EXPECTED_N:
        raise ValueError("Table 1 asset row count mismatch")
    records_path = receipt_dir / "asset_records.jsonl"
    records_sha = TABLE2.sha256_file(records_path)
    hashes: dict[str, str] = {}
    statuses: dict[str, str] = {}
    with records_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            asset_id = str(record["asset_id"])
            if record.get("status") != "EVALUATED" or not record.get("primary_urdf_sha256"):
                raise ValueError(f"Table 1 record not fully evaluated: {asset_id}")
            hashes[asset_id] = str(record["primary_urdf_sha256"])
            statuses[asset_id] = str(record["status"])
    if len(hashes) != EXPECTED_N:
        raise ValueError("Table 1 asset record count mismatch")
    enriched: list[dict[str, Any]] = []
    for row in rows:
        asset_id = str(row["asset_id"])
        enriched.append({**row, "primary_urdf_sha256": hashes[asset_id]})
    return {
        "manifest": manifest,
        "manifest_sha256": manifest_sha,
        "asset_records_sha256": records_sha,
        "rows": sorted(enriched, key=lambda row: int(row["selection_rank"])),
        "release_asset_count": manifest["N_release"],
        "release_category_count": manifest["release_raw_category_count"],
    }


def parent_failure(job: dict[str, Any], reason: str, *, status: str) -> dict[str, Any]:
    failed = TABLE2.failed_record(
        job["asset_id"],
        Path(job["package"]),
        reason,
        primary_urdf_relative_path=job["primary_urdf_relative_path"],
    )
    failed["status"] = status
    failed["model_urdf_sha256"] = job["model_urdf_sha256"]
    failed["primary_urdf_sha256"] = job["primary_urdf_sha256"]
    failed["package_content_manifest_sha256"] = job["package_binding"][
        "content_manifest_sha256"
    ]
    failed["manifest_content_sha256"] = job["manifest_content_sha256"]
    failed["result_origin"] = "parent_synthesized"
    for field in OURS_METADATA_FIELDS:
        if field in job:
            failed[field] = job[field]
    return failed


def validate_audit_result(job: dict[str, Any], record: dict[str, Any]) -> None:
    if record.get("asset_id") != job["asset_id"]:
        raise ValueError("record asset ID mismatch")
    if record.get("status") not in {"completed", "error", "timeout"}:
        raise ValueError(f"record status is invalid: {record.get('status')!r}")
    metrics = record.get("metrics")
    if not isinstance(metrics, dict) or set(metrics) != set(TABLE2.METRIC_NAMES):
        raise ValueError("record metric schema mismatch")
    for name in TABLE2.METRIC_NAMES:
        value = metrics[name]
        if not isinstance(value, dict) or not isinstance(value.get("pass"), bool):
            raise ValueError(f"record metric pass is not boolean: {name}")
    recomputed = all(
        metrics[name]["pass"] for name in TABLE2.METRIC_NAMES if name != "strict_urdf_pass"
    )
    if metrics["strict_urdf_pass"]["pass"] is not recomputed:
        raise ValueError("strict metric is inconsistent")
    if record.get("strict_urdf_pass") is not recomputed:
        raise ValueError("strict compatibility field is inconsistent")
    if record.get("model_urdf_sha256") != job["model_urdf_sha256"]:
        raise ValueError("record binding mismatch: model_urdf_sha256")
    if (
        record.get("package_content_manifest_sha256")
        != job["package_binding"]["content_manifest_sha256"]
    ):
        raise ValueError("record binding mismatch: package_content_manifest_sha256")
    if record.get("manifest_content_sha256") != job["manifest_content_sha256"]:
        raise ValueError("record binding mismatch: manifest_content_sha256")
    if record.get("result_origin") not in {"child_attested", "parent_synthesized"}:
        raise ValueError("record origin is invalid")


def process_group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def terminate_owned_process_group(pgid: int, process: subprocess.Popen[bytes] | None) -> str:
    termination = "none_required"
    if process is not None and process.poll() is None:
        try:
            os.killpg(pgid, 15)
            termination = "sigterm"
        except ProcessLookupError:
            return "already_exited"
        try:
            process.wait(timeout=CHILD_GRACE_SECONDS)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(pgid, 9)
                termination = "sigkill_after_grace"
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                termination = "unterminated"
    elif process_group_exists(pgid):
        try:
            os.killpg(pgid, 15)
            termination = "sigterm_orphaned_group"
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + CHILD_GRACE_SECONDS
        while process_group_exists(pgid) and time.monotonic() < deadline:
            time.sleep(0.01)
        if process_group_exists(pgid):
            try:
                os.killpg(pgid, 9)
                termination = "sigkill_orphaned_group"
            except ProcessLookupError:
                pass
    return termination


def execute_audit_jobs(
    jobs: list[dict[str, Any]],
    *,
    worker_scratch: Path,
    timeout_seconds: float,
    max_workers: int,
    on_result: Any,
) -> list[dict[str, Any]]:
    """Run fresh-interpreter audit jobs with owned-process-group timeouts."""
    if timeout_seconds <= 0 or max_workers <= 0:
        raise ValueError("timeout_seconds and max_workers must be positive")
    if worker_scratch.exists():
        raise FileExistsError(f"worker scratch already exists: {worker_scratch}")
    worker_scratch.mkdir(parents=False)
    worker_scratch = worker_scratch.resolve(strict=True)
    pending = list(enumerate(jobs))
    active: dict[
        int, tuple[int, dict[str, Any], subprocess.Popen[bytes], Path, Path, Path, float]
    ] = {}
    results: dict[int, dict[str, Any]] = {}

    try:
        while pending or active:
            while pending and len(active) < max_workers:
                index, job = pending.pop(0)
                job_root = worker_scratch / f"job_{index:06d}"
                stderr_handle = None
                process: subprocess.Popen[bytes] | None = None
                try:
                    job_root.mkdir()
                    job_path = job_root / "job.json"
                    result_path = job_root / "result.json"
                    stderr_path = job_root / "stderr.log"
                    _atomic_write_json(job_path, job)
                    environment = dict(os.environ)
                    environment.update(TABLE2.CHILD_THREAD_ENVIRONMENT)
                    stderr_handle = stderr_path.open("xb")
                    process = subprocess.Popen(
                        [
                            sys.executable,
                            str(SCRIPT_PATH),
                            "--internal-child-job",
                            str(job_path),
                            "--internal-child-result",
                            str(result_path),
                        ],
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL,
                        stderr=stderr_handle,
                        env=environment,
                        start_new_session=True,
                    )
                except Exception as exc:  # noqa: BLE001
                    if stderr_handle is not None:
                        stderr_handle.close()
                    if process is not None:
                        terminate_owned_process_group(process.pid, process)
                    if job_root.is_dir():
                        shutil.rmtree(job_root)
                    record = parent_failure(
                        job, f"child_spawn_failed: {type(exc).__name__}: {exc}", status="error"
                    )
                    results[index] = record
                    on_result(record)
                    continue
                stderr_handle.close()
                active[process.pid] = (
                    index,
                    job,
                    process,
                    job_root,
                    result_path,
                    stderr_path,
                    time.monotonic(),
                )

            emitted = False
            for pid, state in list(active.items()):
                index, job, process, job_root, result_path, stderr_path, started = state
                returncode = process.poll()
                timed_out = returncode is None and time.monotonic() - started >= timeout_seconds
                if returncode is None and not timed_out:
                    continue
                termination = None
                if timed_out:
                    termination = terminate_owned_process_group(pid, process)
                    returncode = process.returncode
                stderr_evidence = TABLE2.bounded_file_tail(stderr_path)
                stderr_tail = stderr_evidence["stderr_tail"]
                if timed_out:
                    record = parent_failure(job, f"asset_timeout_after_{timeout_seconds}s: {stderr_tail}", status="timeout")
                elif returncode == RUNTIME_BINDING_FATAL_EXIT_CODE:
                    raise FatalRuntimeBindingError(
                        f"child runtime binding failed for {job.get('asset_id')}: {stderr_tail}"
                    )
                elif returncode != 0:
                    record = parent_failure(job, f"child_exit_{returncode}: {stderr_tail}", status="error")
                elif not result_path.is_file():
                    record = parent_failure(job, f"child_result_missing: {stderr_tail}", status="error")
                else:
                    try:
                        record = json.loads(result_path.read_text(encoding="utf-8"))
                        validate_audit_result(job, record)
                    except FatalRuntimeBindingError:
                        raise
                    except Exception as exc:  # noqa: BLE001
                        record = parent_failure(
                            job, f"child_result_invalid: {type(exc).__name__}: {exc}", status="error"
                        )
                if not timed_out and process_group_exists(pid):
                    termination = terminate_owned_process_group(pid, None)
                record["worker_evidence"] = {
                    "pid": pid,
                    "returncode": returncode,
                    "termination": termination,
                    **stderr_evidence,
                    "fresh_interpreter": True,
                    "start_new_session": True,
                }
                del active[pid]
                shutil.rmtree(job_root)
                results[index] = record
                on_result(record)
                emitted = True
            if active and not emitted:
                time.sleep(0.005)
    finally:
        for (_index, _job, process, job_root, _result_path, _stderr_path, _started) in active.values():
            terminate_owned_process_group(process.pid, process)
            if job_root.parent == worker_scratch and job_root.name.startswith("job_"):
                shutil.rmtree(job_root, ignore_errors=True)
        if worker_scratch.is_dir() and not any(worker_scratch.iterdir()):
            worker_scratch.rmdir()
    return [results[index] for index in range(len(jobs))]


def _check_child_bindings(job: dict[str, Any]) -> None:
    binding = job.get("evaluator_binding")
    if not isinstance(binding, dict):
        raise FatalRuntimeBindingError("job evaluator binding missing")
    live = evaluator_binding()
    for key in ("evaluator_path", "evaluator_sha256", "runner_path", "runner_sha256"):
        if binding.get(key) != live.get(key):
            raise FatalRuntimeBindingError(f"evaluator binding mismatch: {key}")
    environment = job.get("environment_binding")
    if not isinstance(environment, dict):
        raise FatalRuntimeBindingError("job environment binding missing")
    live_environment = environment_binding()
    if environment != live_environment:
        raise FatalRuntimeBindingError("environment binding mismatch")


def run_child(job_path: Path, result_path: Path) -> int:
    job_path = job_path.resolve(strict=True)
    result_path = result_path.resolve(strict=False)
    if job_path.name != "job.json" or result_path.name != "result.json":
        raise RuntimeError("internal child paths must use exact job.json/result.json names")
    job = json.loads(job_path.read_text(encoding="utf-8"))
    if not isinstance(job, dict) or not isinstance(job.get("asset_id"), str):
        raise RuntimeError("internal child job schema is invalid")

    try:
        _check_child_bindings(job)
    except FatalRuntimeBindingError as exc:
        print(f"FATAL_RUNTIME_BINDING: {exc}", file=sys.stderr)
        return RUNTIME_BINDING_FATAL_EXIT_CODE

    def finish(record: dict[str, Any]) -> dict[str, Any]:
        record["result_origin"] = "child_attested"
        record["model_urdf_sha256"] = job["model_urdf_sha256"]
        record["primary_urdf_sha256"] = job["primary_urdf_sha256"]
        record["package_content_manifest_sha256"] = job["package_binding"][
            "content_manifest_sha256"
        ]
        record["manifest_content_sha256"] = job["manifest_content_sha256"]
        for field in OURS_METADATA_FIELDS:
            if field in job:
                record[field] = job[field]
        return record

    package = Path(job["package"])
    expected_binding = job["package_binding"]
    try:
        before_binding = TABLE2.package_binding(package)
    except Exception as exc:  # noqa: BLE001
        record = parent_failure(job, f"source_precheck_failed: {type(exc).__name__}: {exc}", status="error")
        record["result_origin"] = "child_attested"
        _atomic_write_json(result_path, record)
        return 0
    if before_binding != expected_binding:
        record = parent_failure(job, "source_changed_before_audit", status="error")
        record["result_origin"] = "child_attested"
        _atomic_write_json(result_path, record)
        return 0

    record = TABLE2.audit_asset_package(
        package,
        run_standard_parser=bool(job["run_standard_parser"]),
        asset_id=job["asset_id"],
        primary_urdf_relative_path=job["primary_urdf_relative_path"],
    )

    try:
        after_binding = TABLE2.package_binding(package)
    except Exception as exc:  # noqa: BLE001
        record = parent_failure(job, f"source_postcheck_failed: {type(exc).__name__}: {exc}", status="error")
        record["result_origin"] = "child_attested"
        _atomic_write_json(result_path, record)
        return 0
    if after_binding != expected_binding:
        record = parent_failure(job, "source_changed_during_audit", status="error")
        record["result_origin"] = "child_attested"
        _atomic_write_json(result_path, record)
        return 0

    _atomic_write_json(result_path, finish(record))
    return 0


def build_jobs(
    rows: list[dict[str, Any]],
    dataset_root: Path,
    *,
    manifest_content_sha256: str,
    run_standard_parser: bool,
) -> list[dict[str, Any]]:
    evaluator = evaluator_binding()
    environment = environment_binding()
    jobs: list[dict[str, Any]] = []
    for row in rows:
        asset_id = str(row["asset_id"])
        package = (dataset_root / str(row["asset_root"])).resolve(strict=True)
        package_binding = TABLE2.package_binding(package)
        urdf_sha256 = TABLE2.sha256_file(package / "model.urdf")
        if urdf_sha256 != str(row["primary_urdf_sha256"]):
            raise ValueError(f"model.urdf hash mismatch for {asset_id}")
        jobs.append(
            {
                "asset_id": asset_id,
                "raw_category": str(row["raw_category"]),
                "seed_name": str(row["seed_name"]),
                "asset_root": str(row["asset_root"]),
                "primary_urdf": str(row["primary_urdf"]),
                "table1_selection_rank": int(row["selection_rank"]),
                "package": str(package),
                "package_binding": package_binding,
                "run_standard_parser": run_standard_parser,
                "primary_urdf_relative_path": "model.urdf",
                "model_urdf_sha256": urdf_sha256,
                "primary_urdf_sha256": urdf_sha256,
                "manifest_content_sha256": manifest_content_sha256,
                "evaluator_binding": evaluator,
                "environment_binding": environment,
            }
        )
    return jobs


def summary_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    lines = [
        "# Ours-500K Table 2 URDF audit",
        "",
        f"Run classification: **{manifest['classification']}**.",
        "",
        (
            f"Frozen cohort: N={summary['n_eval']}, full acquired Table 1 roster order "
            "(selection_rank ascending); no resampling/reselection."
        ),
        "",
        "| Metric | Result |",
        "|---|---:|",
    ]
    for name in TABLE2.METRIC_NAMES:
        result = summary["metrics"][name]
        percentage = "N/A" if result["rate"] is None else f"{100 * result['rate']:.2f}%"
        lines.append(f"| {name} | {result['passed']} / {result['denominator']} ({percentage}) |")
    category_macro = summary["category_macro"]
    if category_macro.get("state") == "evaluated":
        lines.extend(
            [
                "",
                (
                    "Category macro average: evaluated over "
                    f"{category_macro['category_count']} observed {category_macro['category_field']} groups "
                    "using an unweighted mean. This frozen cohort is the full acquired roster, "
                    "not category-balanced against other datasets."
                ),
                "",
                "| Metric | Category macro |",
                "|---|---:|",
            ]
        )
        for name in TABLE2.METRIC_NAMES:
            rate = category_macro["metrics"][name]["rate"]
            lines.append(f"| {name} | {'N/A' if rate is None else f'{100 * rate:.2f}%'} |")
    lines.append("")
    return "\n".join(lines)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve(strict=True)
    started_at = utc_now()

    archive_binding = TABLE1O.verify_archive_binding(
        args.archive, args.archive_sha256_file
    )
    roster = TABLE1O.build_release_roster(dataset_root)
    roster_sha256 = TABLE1O.freeze_roster(roster, archive_binding)
    if roster_sha256 != EXPECTED_ROSTER_SHA256:
        raise ValueError(
            f"on-disk roster hash mismatch: {roster_sha256} != {EXPECTED_ROSTER_SHA256}"
        )
    cohort = load_table1_cohort(args.table1_receipt)
    rows = cohort["rows"][: args.limit] if args.limit is not None else cohort["rows"]
    expected_rows = args.limit if args.limit is not None else EXPECTED_N
    if len(rows) != expected_rows:
        raise ValueError("frozen cohort row count mismatch")

    environment = TABLE2.environment_metadata(
        not args.no_standard_parser, args.workers
    )
    config = {
        **TABLE2.EVALUATOR_CONFIG,
        "dataset": DATASET_LABEL,
        "workers": args.workers,
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "selection_algorithm": (
            "existing frozen Table 1 Ours-500K full-roster manifest order "
            "(selection_rank ascending); no resampling/reselection"
        ),
        "package_staging_policy": (
            "no staging: release layout is already a self-contained package per asset "
            "(<category>/<seed_N>/model.urdf plus assets/ resource closure); packages are "
            "audited in place with content binding re-verified before and after each audit"
        ),
    }
    protocol_binding = TABLE2.freeze_protocol_snapshot(output)
    evaluation = {
        "command": [str(SCRIPT_PATH), *sys.argv[1:]],
        "config": config,
        "config_sha256": TABLE2.canonical_sha256(config),
        "asset_timeout_seconds": args.asset_timeout_seconds,
        "workers": args.workers,
        "metrics": list(TABLE2.METRIC_NAMES),
        "denominator_policy": config["denominator_policy"],
        "parse_rate_scope": config["parse_rate_scope"],
        "resource_validation_scope": config["resource_validation_scope"],
        "evaluator_path": str(TABLE2_EVALUATOR_PATH.resolve()),
        "evaluator_sha256": TABLE2.sha256_file(TABLE2_EVALUATOR_PATH.resolve()),
        "runner_path": str(SCRIPT_PATH),
        "runner_sha256": TABLE2.sha256_file(SCRIPT_PATH),
        "table1_cohort_loader_path": str(TABLE1_RUNNER_PATH.resolve()),
        "table1_cohort_loader_sha256": TABLE2.sha256_file(TABLE1_RUNNER_PATH.resolve()),
        "environment": environment,
        "environment_sha256": TABLE2.canonical_sha256(environment),
        **protocol_binding,
    }

    assets = []
    for row in rows:
        asset_id = str(row["asset_id"])
        package = (dataset_root / str(row["asset_root"])).resolve(strict=True)
        assets.append(
            {
                "selection_index": int(row["selection_rank"]) - 1,
                "asset_id": asset_id,
                "raw_category": str(row["raw_category"]),
                "seed_name": str(row["seed_name"]),
                "asset_root": str(row["asset_root"]),
                "primary_urdf": str(row["primary_urdf"]),
                "primary_urdf_sha256": str(row["primary_urdf_sha256"]),
                "package": str(package),
            }
        )

    run_manifest = {
        "schema_version": "1.0.0-ours-500k",
        "created_at": started_at,
        "dataset": DATASET_LABEL,
        "mode": "formal" if args.limit is None else "smoke",
        "classification": "FORMAL" if args.limit is None else "SMOKE",
        "source": {
            "dataset_root": str(dataset_root),
            "table1_receipt": str(args.table1_receipt.resolve(strict=True)),
            "table1_manifest_sha256": cohort["manifest_sha256"],
            "table1_asset_records_sha256": cohort["asset_records_sha256"],
            "table1_roster_sha256": cohort["manifest"]["roster_sha256"],
            "table1_protocol_sha256": cohort["manifest"]["protocol_sha256"],
            "cohort_type": TABLE1O.COHORT_TYPE,
            "archive_binding": archive_binding,
            "on_disk_roster_sha256": roster_sha256,
            "roster_reverified": True,
        },
        "evaluation": evaluation,
        "n_requested": expected_rows,
        "n_eval": len(rows),
        "assets": assets,
    }
    run_manifest["manifest_content_sha256"] = TABLE2.manifest_self_hash(run_manifest)
    TABLE2.atomic_write_json(output / "manifest.json", run_manifest)
    TABLE2.atomic_write_json(output / "environment.json", environment)

    jobs = build_jobs(
        rows,
        dataset_root,
        manifest_content_sha256=run_manifest["manifest_content_sha256"],
        run_standard_parser=not args.no_standard_parser,
    )
    worker_scratch = output / ".worker_scratch"
    records = execute_audit_jobs(
        jobs,
        worker_scratch=worker_scratch,
        timeout_seconds=args.asset_timeout_seconds,
        max_workers=args.workers,
        on_result=lambda record: None,
    )

    TABLE2.atomic_write_jsonl(output / "asset_records.jsonl", records)
    summary = TABLE2.aggregate_records(records, len(rows), category_field="raw_category")
    summary["dataset"] = DATASET_LABEL
    summary["classification"] = run_manifest["classification"]
    summary["created_at"] = started_at
    summary["completed_at"] = utc_now()
    summary["cohort"] = {
        "N_release": cohort["release_asset_count"],
        "N_eval": len(rows),
        "release_raw_categories": cohort["release_category_count"],
        "eval_raw_categories": len({str(row["raw_category"]) for row in rows}),
        "cohort_type": TABLE1O.COHORT_TYPE,
        "table1_manifest_sha256": cohort["manifest_sha256"],
    }
    TABLE2.atomic_write_json(output / "summary.json", summary)
    TABLE2.atomic_write_text(output / "summary.md", summary_markdown(summary, run_manifest))

    artifact_files = (
        "manifest.json",
        "asset_records.jsonl",
        "summary.json",
        "summary.md",
        "environment.json",
        TABLE2.PROTOCOL_SNAPSHOT_NAME,
    )
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": utc_now(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": TABLE2.sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    TABLE2.atomic_write_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve()
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the dataset root")

    with SHARED._output_lock(output):
        staging = output.parent / (f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}")
        staging.mkdir(parents=False, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            SHARED._verify_staged_artifacts(staging)
            SHARED._publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument(
        "--archive-sha256-file", type=Path, default=DEFAULT_ARCHIVE_SHA256_FILE
    )
    parser.add_argument("--table1-receipt", type=Path, default=DEFAULT_TABLE1_RECEIPT)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--asset-timeout-seconds", type=float, default=ASSET_TIMEOUT_SECONDS)
    parser.add_argument("--no-standard-parser", action="store_true")
    return parser.parse_args(argv)


def main() -> int:
    effective_argv = list(sys.argv[1:])
    if effective_argv and effective_argv[0] == "--internal-child-job":
        if len(effective_argv) != 4 or effective_argv[2] != "--internal-child-result":
            print("ERROR: invalid internal child invocation", file=sys.stderr)
            return 2
        try:
            return run_child(Path(effective_argv[1]), Path(effective_argv[3]))
        except FatalRuntimeBindingError as exc:
            print(f"FATAL_RUNTIME_BINDING: {exc}", file=sys.stderr)
            return RUNTIME_BINDING_FATAL_EXIT_CODE
        except Exception as exc:  # noqa: BLE001
            print(f"ERROR: internal child failed: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1

    args = parse_args(effective_argv)
    if args.workers <= 0 or args.asset_timeout_seconds <= 0:
        raise SystemExit("--workers and --asset-timeout-seconds must be positive")
    if args.limit is not None and args.limit <= 0:
        raise SystemExit("--limit must be positive")
    if args.output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        suffix = f"smoke_n{args.limit}" if args.limit is not None else "n500"
        args.output = (
            REPO
            / f"exp/runtime/table2_urdf_ours_500k_table1cohort_{suffix}_{stamp}"
        )
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "n_eval": summary["n_eval"],
                "strict_urdf_pass": summary["strict_urdf_pass"]["passed"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
