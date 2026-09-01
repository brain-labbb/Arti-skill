#!/usr/bin/env python3
"""Incrementally evaluate the 791 Articraft GitHub-only records.

The pipeline keeps the released 9,996-asset receipts immutable.  It compiles
and evaluates only the GitHub-only records, then concatenates the atomic
records and re-runs the Table 1--4 aggregators against a new 10,787-asset
roster.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable, Mapping, Sequence

import table123_full_release_common as common
from build_table123_full_release_rosters import _row as build_roster_row


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
EXP_ROOT = REPO_ROOT / "exp"
GITHUB_ROOT = EXP_ROOT / "Articraft-10K-github"
RELEASE_ROOT = EXP_ROOT / "Articraft-10K" / "released_urdf"
OLD_ROOT = EXP_ROOT / "runtime" / "table123_full_release_20260825"
OLD_ROSTER = OLD_ROOT / "articraft" / "full_release_manifest.json"
OLD_TABLE4 = EXP_ROOT / "runtime" / "table4_full_release_20260826" / "articraft"
PAPER_HARNESS = EXP_ROOT / "baselines" / "Articraft-paper-harness"
COMPILE_PYTHON = PAPER_HARNESS / ".venv" / "bin" / "python"
COMPILE_SCRIPT = PAPER_HARNESS / "cli" / "compile_record.py"
COMPILE_COMPAT_SCRIPT = SCRIPT.with_name("articraft_compile_compat.py")
EVAL_PYTHON = EXP_ROOT / ".venv_low_medium" / "bin" / "python"
DEFAULT_OUTPUT = EXP_ROOT / "runtime" / "articraft_github_merged_10787_20260827"

EXPECTED_GITHUB = 10_787
EXPECTED_RELEASED = 9_996
EXPECTED_SUPPLEMENT = 791
EXPECTED_CATEGORIES = 244
BLAS_ENVIRONMENT = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def utc_now() -> str:
    import datetime as dt

    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def atomic_text(path: Path, text: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to replace symlink: {path}")
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, value: Any) -> None:
    atomic_text(
        path,
        json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True, allow_nan=False)
        + "\n",
    )


def atomic_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to replace symlink: {path}")
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            for row in rows:
                stream.write(
                    json.dumps(
                        dict(row),
                        sort_keys=True,
                        separators=(",", ":"),
                        ensure_ascii=True,
                        allow_nan=False,
                    )
                    + "\n"
                )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError(f"invalid JSONL row {line_number}: {path}")
            rows.append(row)
    return rows


def load_roster_metadata(path: Path) -> dict[str, Any]:
    """Validate a frozen roster receipt without re-reading package payloads."""

    manifest = read_json(path)
    if manifest.get("schema_version") != common.SCHEMA_VERSION:
        raise ValueError(f"roster schema mismatch: {path}")
    declared_manifest_hash = manifest.get("manifest_content_sha256")
    hash_input = dict(manifest)
    hash_input.pop("manifest_content_sha256", None)
    if declared_manifest_hash != canonical_sha256(hash_input):
        raise ValueError(f"roster manifest self-hash mismatch: {path}")
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ValueError(f"roster rows missing: {path}")
    if manifest.get("roster_sha256") != canonical_sha256(rows):
        raise ValueError(f"roster content hash mismatch: {path}")
    ids: list[str] = []
    joint_total = 0
    for ordinal, row in enumerate(rows):
        if not isinstance(row, Mapping) or row.get("ordinal") != ordinal:
            raise ValueError(f"roster row order mismatch at {ordinal}: {path}")
        record_id = str(row.get("asset_id", ""))
        if not record_id:
            raise ValueError(f"roster asset identity missing at {ordinal}: {path}")
        ids.append(record_id)
        joint_total += int(row.get("joint_count", 0))
    if len(ids) != len(set(ids)) or manifest.get("N_eval") != len(rows):
        raise ValueError(f"roster asset denominator mismatch: {path}")
    if manifest.get("J_eval") != joint_total:
        raise ValueError(f"roster joint denominator mismatch: {path}")
    roster_jsonl = Path(path).with_name("full_release_roster.jsonl")
    if manifest.get("roster_jsonl_sha256") != sha256_file(roster_jsonl):
        raise ValueError(f"roster JSONL hash mismatch: {path}")
    return manifest


def load_module(path: Path, name: str) -> Any:
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def git_commit(path: Path) -> str:
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def active_model_path(record_dir: Path, record: Mapping[str, Any]) -> Path:
    artifacts = record.get("artifacts")
    if isinstance(artifacts, Mapping) and isinstance(artifacts.get("model_py"), str):
        return record_dir / str(artifacts["model_py"])
    revision = str(record.get("active_revision_id") or "rev_000001")
    return record_dir / "revisions" / revision / "model.py"


def source_inventory(output: Path, *, create_staging: bool) -> dict[str, Any]:
    index_path = GITHUB_ROOT / "records_index.jsonl"
    cohort_dir = output / "cohort"
    frozen_path = cohort_dir / "source_inventory.json"
    ids_path = cohort_dir / "github_only_ids.txt"
    categories_path = cohort_dir / "categories.json"
    model_hashes_path = cohort_dir / "supplement_model_hashes.json"
    if all(path.is_file() for path in (frozen_path, ids_path, categories_path, model_hashes_path)):
        inventory = read_json(frozen_path)
        ids = [line for line in ids_path.read_text(encoding="utf-8").splitlines() if line]
        categories = read_json(categories_path)
        model_hashes = read_json(model_hashes_path)
        stable = (
            inventory.get("schema_version") == "articraft_github_incremental_source_inventory_v1"
            and inventory.get("github_commit") == git_commit(GITHUB_ROOT)
            and inventory.get("github_index_sha256") == sha256_file(index_path)
            and inventory.get("released_roster_file_sha256") == sha256_file(OLD_ROSTER)
            and len(ids) == EXPECTED_SUPPLEMENT
            and set(ids) == set(model_hashes)
            and len(categories) == EXPECTED_GITHUB
        )
        if not stable:
            raise ValueError("frozen source inventory no longer matches its inputs")
        if create_staging:
            records_dir = output / "staging" / "data" / "records"
            records_dir.mkdir(parents=True, exist_ok=True)
            for record_id in ids:
                source = (GITHUB_ROOT / "records" / record_id).resolve(strict=True)
                link = records_dir / record_id
                if link.is_symlink():
                    if link.resolve(strict=True) != source:
                        raise ValueError(f"staging symlink changed: {link}")
                    continue
                if link.exists():
                    raise ValueError(f"staging path is not a symlink: {link}")
                link.symlink_to(source, target_is_directory=True)
        return inventory

    index_rows = read_jsonl(index_path)
    by_id: dict[str, dict[str, Any]] = {}
    for row in index_rows:
        record_id = str(row.get("record_id", ""))
        category = str(row.get("category_slug", ""))
        if not record_id or not category or record_id in by_id:
            raise ValueError(f"invalid or duplicate GitHub index row: {record_id!r}")
        by_id[record_id] = row
    old_manifest = common.load_roster(OLD_ROSTER, verify_sources=False)
    old_ids = {str(row["asset_id"]) for row in old_manifest["rows"]}
    github_ids = set(by_id)
    supplement_ids = sorted(github_ids - old_ids)
    missing_from_github = sorted(old_ids - github_ids)
    if len(github_ids) != EXPECTED_GITHUB:
        raise ValueError(f"GitHub index count changed: {len(github_ids)}")
    if len(old_ids) != EXPECTED_RELEASED:
        raise ValueError(f"released roster count changed: {len(old_ids)}")
    if len(supplement_ids) != EXPECTED_SUPPLEMENT or missing_from_github:
        raise ValueError(
            f"unexpected cohort delta: supplement={len(supplement_ids)} "
            f"missing={len(missing_from_github)}"
        )

    def inspect_source(record_id: str) -> tuple[str, str, str | None]:
        record_dir = GITHUB_ROOT / "records" / record_id
        record = read_json(record_dir / "record.json")
        if str(record.get("record_id")) != record_id:
            raise ValueError(f"record metadata identity mismatch: {record_id}")
        model_path = active_model_path(record_dir, record)
        if not model_path.is_file() or model_path.is_symlink():
            raise ValueError(f"missing active model.py: {record_id}")
        observed = sha256_file(model_path)
        declared = record.get("hashes", {}).get("model_py_sha256")
        return record_id, observed, str(declared) if declared else None

    model_hashes: dict[str, str] = {}
    declared_model_hash_mismatches: dict[str, dict[str, str]] = {}
    with ThreadPoolExecutor(max_workers=32) as executor:
        inspected = executor.map(inspect_source, supplement_ids)
        for record_id, observed, declared in inspected:
            if declared and declared != observed:
                declared_model_hash_mismatches[record_id] = {
                    "declared": declared,
                    "observed": observed,
                }
            model_hashes[record_id] = observed

    atomic_text(ids_path, "".join(f"{record_id}\n" for record_id in supplement_ids))
    categories = {record_id: str(by_id[record_id]["category_slug"]) for record_id in by_id}
    category_values = set(categories.values())
    if len(category_values) != EXPECTED_CATEGORIES:
        raise ValueError(f"category count changed: {len(category_values)}")

    inventory = {
        "schema_version": "articraft_github_incremental_source_inventory_v1",
        "created_at": utc_now(),
        "github_root": str(GITHUB_ROOT.resolve()),
        "github_commit": git_commit(GITHUB_ROOT),
        "github_index": str(index_path.resolve()),
        "github_index_sha256": sha256_file(index_path),
        "github_record_count": len(github_ids),
        "github_category_count": len(category_values),
        "released_roster": str(OLD_ROSTER.resolve()),
        "released_roster_file_sha256": sha256_file(OLD_ROSTER),
        "released_roster_sha256": old_manifest["roster_sha256"],
        "released_manifest_content_sha256": old_manifest["manifest_content_sha256"],
        "released_count": len(old_ids),
        "supplement_count": len(supplement_ids),
        "supplement_ids_path": str(ids_path.resolve()),
        "supplement_ids_sha256": sha256_file(ids_path),
        "supplement_model_hashes_sha256": canonical_sha256(model_hashes),
        "declared_model_hash_mismatch_count": len(declared_model_hash_mismatches),
        "declared_model_hash_mismatches_sha256": canonical_sha256(
            declared_model_hash_mismatches
        ),
        "missing_from_github": missing_from_github,
        "compiler": {
            "harness_root": str(PAPER_HARNESS.resolve()),
            "harness_commit": git_commit(PAPER_HARNESS),
            "python": str(COMPILE_PYTHON.resolve()),
            "compile_script": str(COMPILE_SCRIPT.resolve()),
            "compile_script_sha256": sha256_file(COMPILE_SCRIPT),
            "target": "full",
            "validate": True,
            "strict_geom_qc": True,
        },
    }
    inventory["inventory_content_sha256"] = canonical_sha256(inventory)
    atomic_json(cohort_dir / "source_inventory.json", inventory)
    atomic_json(categories_path, categories)
    atomic_json(model_hashes_path, model_hashes)
    atomic_json(
        cohort_dir / "declared_model_hash_mismatches.json",
        declared_model_hash_mismatches,
    )

    if create_staging:
        records_dir = output / "staging" / "data" / "records"
        records_dir.mkdir(parents=True, exist_ok=True)
        for record_id in supplement_ids:
            source = (GITHUB_ROOT / "records" / record_id).resolve(strict=True)
            link = records_dir / record_id
            if link.is_symlink():
                if link.resolve(strict=True) != source:
                    raise ValueError(f"staging symlink changed: {link}")
                continue
            if link.exists():
                raise ValueError(f"staging path is not a symlink: {link}")
            link.symlink_to(source, target_is_directory=True)
    return inventory


def compile_success(output: Path, record_id: str, model_hash: str) -> tuple[bool, str]:
    package = output / "staging" / "data" / "cache" / "record_materialization" / record_id
    report_path = package / "compile_report.json"
    urdf_path = package / "model.urdf"
    if not report_path.is_file() or not urdf_path.is_file():
        return False, "missing compile report or model.urdf"
    try:
        report = read_json(report_path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return False, f"invalid compile report: {error}"
    metrics = report.get("metrics") if isinstance(report.get("metrics"), Mapping) else {}
    fingerprints = (
        metrics.get("fingerprint_inputs")
        if isinstance(metrics.get("fingerprint_inputs"), Mapping)
        else {}
    )
    checks = (
        str(report.get("status")) == "success",
        str(report.get("record_id")) == record_id,
        str(metrics.get("compile_level")) == "full",
        str(metrics.get("validation_level")) == "full",
        str(fingerprints.get("model_py_sha256")) == model_hash,
    )
    if not all(checks):
        return False, "compile report does not match the frozen full/validated source"
    return True, "success"


def materialization_ready(
    output: Path, record_id: str, model_hash: str
) -> tuple[bool, str]:
    success, reason = compile_success(output, record_id, model_hash)
    if success:
        return True, reason
    package = output / "staging" / "data" / "cache" / "record_materialization" / record_id
    marker_path = package / "materialization_failure.json"
    urdf_path = package / "model.urdf"
    if not marker_path.is_file() or not urdf_path.is_file():
        return False, reason
    try:
        marker = read_json(marker_path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return False, f"invalid retained-failure marker: {error}"
    valid = (
        marker.get("schema_version") == "articraft_source_compile_failure_v1"
        and marker.get("record_id") == record_id
        and marker.get("source_model_py_sha256") == model_hash
        and marker.get("placeholder_urdf_sha256") == sha256_file(urdf_path)
    )
    return (True, "retained_source_compile_failure") if valid else (
        False,
        "retained-failure marker binding mismatch",
    )


def retain_compile_failure(
    output: Path, record_id: str, model_hash: str, reason: str
) -> dict[str, Any]:
    package = output / "staging" / "data" / "cache" / "record_materialization" / record_id
    package.mkdir(parents=True, exist_ok=True)
    urdf_path = package / "model.urdf"
    atomic_text(
        urdf_path,
        f"ARTICRAFT_SOURCE_COMPILE_FAILED record_id={record_id}\n",
    )
    report_path = package / "compile_report.json"
    marker = {
        "schema_version": "articraft_source_compile_failure_v1",
        "record_id": record_id,
        "status": "retained_source_compile_failure",
        "reason": reason,
        "source_model_py_sha256": model_hash,
        "compile_report_sha256": sha256_file(report_path) if report_path.is_file() else None,
        "placeholder_policy": (
            "intentionally malformed model.urdf keeps the asset in N_eval and "
            "forces Table 1--4 to retain a fail-closed record"
        ),
        "placeholder_urdf_sha256": sha256_file(urdf_path),
        "created_at": utc_now(),
    }
    marker["marker_content_sha256"] = canonical_sha256(marker)
    atomic_json(package / "materialization_failure.json", marker)
    return marker


def compile_one(
    output: Path,
    record_id: str,
    model_hash: str,
    *,
    timeout_seconds: float,
) -> dict[str, Any]:
    success, reason = compile_success(output, record_id, model_hash)
    if success:
        return {"record_id": record_id, "status": "success", "origin": "resume"}
    logs = output / "compile" / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    command = [
        str(COMPILE_PYTHON),
        str(COMPILE_SCRIPT),
        record_id,
        "--repo-root",
        str(PAPER_HARNESS),
        "--data-dir",
        str(output / "staging" / "data"),
        "--target",
        "full",
        "--validate",
        "--strict-geom-qc",
    ]
    environment = os.environ.copy()
    environment.update(BLAS_ENVIRONMENT)
    started = time.monotonic()
    try:
        result = subprocess.run(
            command,
            cwd=PAPER_HARNESS,
            env=environment,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
        returncode = result.returncode
        stdout = result.stdout
        stderr = result.stderr
    except subprocess.TimeoutExpired as error:
        returncode = None
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else error.stdout or ""
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else error.stderr or ""
        reason = f"timeout after {timeout_seconds:g} seconds"
    atomic_text(logs / f"{record_id}.stdout.txt", stdout)
    atomic_text(logs / f"{record_id}.stderr.txt", stderr)
    success, validation_reason = compile_success(output, record_id, model_hash)
    status = "success" if returncode == 0 and success else "failed"
    return {
        "record_id": record_id,
        "status": status,
        "origin": "executed",
        "returncode": returncode,
        "elapsed_seconds": time.monotonic() - started,
        "reason": validation_reason if status == "failed" else None,
        "preexisting_reason": reason,
    }


def run_compile(output: Path, *, workers: int, timeout_seconds: float) -> dict[str, Any]:
    source_inventory(output, create_staging=True)
    ids = [line for line in (output / "cohort" / "github_only_ids.txt").read_text().splitlines() if line]
    model_hashes = read_json(output / "cohort" / "supplement_model_hashes.json")
    records: dict[str, dict[str, Any]] = {}
    status_path = output / "compile" / "compile_status.json"
    if status_path.is_file():
        previous = read_json(status_path)
        records = {
            str(row["record_id"]): dict(row)
            for row in previous.get("records", [])
            if isinstance(row, Mapping) and row.get("record_id") in ids
        }
    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {
            executor.submit(
                compile_one,
                output,
                record_id,
                str(model_hashes[record_id]),
                timeout_seconds=timeout_seconds,
            ): record_id
            for record_id in ids
        }
        for completed, future in enumerate(as_completed(futures), 1):
            record_id = futures[future]
            try:
                records[record_id] = future.result()
            except BaseException as error:  # retain every attempted source
                records[record_id] = {
                    "record_id": record_id,
                    "status": "failed",
                    "origin": "parent_exception",
                    "reason": f"{type(error).__name__}: {error}",
                }
            if completed % 16 == 0 or completed == len(futures):
                ordered = [records[record_id] for record_id in ids if record_id in records]
                payload = {
                    "schema_version": "articraft_github_incremental_compile_status_v1",
                    "updated_at": utc_now(),
                    "workers": workers,
                    "timeout_seconds": timeout_seconds,
                    "completed": len(ordered),
                    "expected": len(ids),
                    "success": sum(row.get("status") == "success" for row in ordered),
                    "failed": sum(row.get("status") != "success" for row in ordered),
                    "elapsed_seconds": time.monotonic() - started,
                    "records": ordered,
                }
                payload["status_content_sha256"] = canonical_sha256(payload)
                atomic_json(status_path, payload)
                print(
                    f"compile {payload['completed']}/{payload['expected']} "
                    f"success={payload['success']} failed={payload['failed']}",
                    flush=True,
                )
    final = read_json(status_path)
    return final


def compile_fallback_one(
    output: Path,
    record_id: str,
    model_hash: str,
    *,
    timeout_seconds: float,
    fallback_required: bool,
) -> dict[str, Any]:
    success, _reason = compile_success(output, record_id, model_hash)
    if success:
        return {
            "record_id": record_id,
            "status": "success",
            "origin": "compatibility_fallback_resume" if fallback_required else "strict_compile",
            "compile_policy": (
                "full_validate_non_strict_geom_qc"
                if fallback_required
                else "full_validate_strict_geom_qc"
            ),
        }
    command = [
        str(COMPILE_PYTHON),
        str(COMPILE_COMPAT_SCRIPT),
        record_id,
        "--repo-root",
        str(PAPER_HARNESS),
        "--data-dir",
        str(output / "staging" / "data"),
        "--target",
        "full",
        "--validate",
    ]
    environment = os.environ.copy()
    environment.update(BLAS_ENVIRONMENT)
    started = time.monotonic()
    try:
        result = subprocess.run(
            command,
            cwd=PAPER_HARNESS,
            env=environment,
            capture_output=True,
            text=True,
            timeout=timeout_seconds,
            check=False,
        )
        returncode = result.returncode
        stdout = result.stdout
        stderr = result.stderr
    except subprocess.TimeoutExpired as error:
        returncode = None
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else error.stdout or ""
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else error.stderr or ""
    logs = output / "compile" / "fallback_logs"
    logs.mkdir(parents=True, exist_ok=True)
    atomic_text(logs / f"{record_id}.stdout.txt", stdout)
    atomic_text(logs / f"{record_id}.stderr.txt", stderr)
    success, reason = compile_success(output, record_id, model_hash)
    return {
        "record_id": record_id,
        "status": "success" if returncode == 0 and success else "failed",
        "origin": "compatibility_fallback",
        "compile_policy": "full_validate_non_strict_geom_qc",
        "compatibility_alias": (
            "warn_if_articulation_origin_near_geometry="
            "warn_if_articulation_origin_far_from_geometry"
        ),
        "returncode": returncode,
        "elapsed_seconds": time.monotonic() - started,
        "reason": None if returncode == 0 and success else reason,
    }


def run_compile_fallback(
    output: Path, *, workers: int, timeout_seconds: float
) -> dict[str, Any]:
    source_inventory(output, create_staging=True)
    ids = [
        line
        for line in (output / "cohort" / "github_only_ids.txt").read_text().splitlines()
        if line
    ]
    model_hashes = read_json(output / "cohort" / "supplement_model_hashes.json")
    strict_status = read_json(output / "compile" / "compile_status.json")
    strict_failed = {
        str(row["record_id"])
        for row in strict_status.get("records", [])
        if isinstance(row, Mapping) and row.get("status") != "success"
    }
    pending = [
        record_id
        for record_id in ids
        if not compile_success(output, record_id, str(model_hashes[record_id]))[0]
    ]
    records: dict[str, dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {
            executor.submit(
                compile_fallback_one,
                output,
                record_id,
                str(model_hashes[record_id]),
                timeout_seconds=timeout_seconds,
                fallback_required=record_id in strict_failed,
            ): record_id
            for record_id in ids
        }
        for completed, future in enumerate(as_completed(futures), 1):
            record_id = futures[future]
            try:
                records[record_id] = future.result()
            except BaseException as error:
                records[record_id] = {
                    "record_id": record_id,
                    "status": "failed",
                    "origin": "parent_exception",
                    "reason": f"{type(error).__name__}: {error}",
                }
            if completed % 16 == 0 or completed == len(futures):
                success_count = sum(row.get("status") == "success" for row in records.values())
                print(
                    f"fallback {completed}/{len(ids)} success={success_count} "
                    f"failed={completed - success_count}",
                    flush=True,
                )
    ordered = [records[record_id] for record_id in ids]
    remaining = [row for row in ordered if row.get("status") != "success"]
    for row in remaining:
        record_id = str(row["record_id"])
        marker = retain_compile_failure(
            output,
            record_id,
            str(model_hashes[record_id]),
            str(row.get("reason") or "source compile failed"),
        )
        row["status"] = "retained_source_compile_failure"
        row["materialization_failure_marker_sha256"] = marker[
            "marker_content_sha256"
        ]
    receipt = {
        "schema_version": "articraft_github_incremental_compile_receipt_v1",
        "completed_at": utc_now(),
        "primary_policy": "full_validate_strict_geom_qc",
        "fallback_policy": "full_validate_non_strict_geom_qc",
        "fallback_reason": (
            "retain all frozen sources in the evaluation denominator; geometry QC "
            "findings remain measurable failures in Table 2/4"
        ),
        "compatibility_wrapper": str(COMPILE_COMPAT_SCRIPT.resolve()),
        "compatibility_wrapper_sha256": sha256_file(COMPILE_COMPAT_SCRIPT),
        "compatibility_aliases": [
            "warn_if_articulation_origin_near_geometry=warn_if_articulation_origin_far_from_geometry",
            "warn_if_part_geometry_disconnected=warn_if_part_contains_disconnected_geometry_islands",
            "check_articulation_overlaps=fail_if_articulation_overlaps",
            "use=visual accepted by legacy warning helpers",
        ],
        "strict_success_count": EXPECTED_SUPPLEMENT - len(strict_failed),
        "fallback_cohort_count": len(strict_failed),
        "fallback_attempt_count_this_run": len(pending),
        "compiled_package_count": len(ordered) - len(remaining),
        "retained_source_compile_failure_count": len(remaining),
        "denominator_ready_count": len(ordered),
        "records": ordered,
    }
    receipt["receipt_content_sha256"] = canonical_sha256(receipt)
    atomic_json(output / "compile" / "compile_receipt.json", receipt)
    return receipt


def copy_package(source: Path, target: Path) -> None:
    if target.is_symlink():
        raise ValueError(f"package target is a symlink: {target}")
    if target.is_dir():
        source_report = source / "compile_report.json"
        target_report = target / "compile_report.json"
        if (
            source_report.is_file()
            and target_report.is_file()
            and sha256_file(source_report) == sha256_file(target_report)
            and (target / "model.urdf").is_file()
        ):
            return
        raise ValueError(f"existing package does not match compile cache: {target}")
    shutil.copytree(source, target, symlinks=False)


def normalize_package_binding(row: dict[str, Any]) -> dict[str, Any]:
    """Canonicalize builder bindings after recursive traversal ordering."""

    files = sorted(
        (dict(item) for item in row.get("package_files", [])),
        key=lambda item: str(item["path"]),
    )
    binding_hash = canonical_sha256(files)
    row["package_files"] = files
    row["package_binding_sha256"] = binding_hash
    package_binding = row.get("package_binding")
    if isinstance(package_binding, Mapping):
        normalized_binding = dict(package_binding)
        normalized_binding["files"] = [
            {
                "path": item["path"],
                "bytes": item["size"],
                "sha256": item["sha256"],
            }
            for item in files
        ]
        normalized_binding["content_manifest_sha256"] = binding_hash
        row["package_binding"] = normalized_binding
    return row


def build_rosters(output: Path, *, workers: int) -> tuple[dict[str, Any], dict[str, Any]]:
    source_inventory(output, create_staging=True)
    ids = [line for line in (output / "cohort" / "github_only_ids.txt").read_text().splitlines() if line]
    categories = read_json(output / "cohort" / "categories.json")
    model_hashes = read_json(output / "cohort" / "supplement_model_hashes.json")
    for record_id in ids:
        success, reason = materialization_ready(
            output, record_id, str(model_hashes[record_id])
        )
        if not success:
            raise RuntimeError(f"cannot freeze uncompiled source {record_id}: {reason}")

    cache_root = output / "staging" / "data" / "cache" / "record_materialization"
    package_root = output / "packages"
    package_root.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        futures = {
            executor.submit(copy_package, cache_root / record_id, package_root / record_id): record_id
            for record_id in ids
        }
        for future in as_completed(futures):
            future.result()

    def make_row(record_id: str) -> dict[str, Any]:
        package = package_root / record_id
        return normalize_package_binding(build_roster_row(
            package / "model.urdf",
            source_root=package_root,
            asset_id=record_id,
            category=str(categories[record_id]),
            package=package,
            cohort_origin="github_source_reconstructed_791",
            source_model_py_sha256=str(model_hashes[record_id]),
        ))

    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        supplement_rows = list(executor.map(make_row, ids))

    inventory = read_json(output / "cohort" / "source_inventory.json")
    bindings = [
        {
            "name": "Articraft-10K-GitHub-source",
            "path": str(GITHUB_ROOT.resolve()),
            "git_commit": inventory["github_commit"],
        },
        {
            "name": "Articraft-10K-GitHub-index",
            "path": str((GITHUB_ROOT / "records_index.jsonl").resolve()),
            "sha256": inventory["github_index_sha256"],
            "bytes": (GITHUB_ROOT / "records_index.jsonl").stat().st_size,
        },
        {"name": "reconstructed_packages", "path": str(package_root.resolve())},
    ]
    supplement_roster_path = output / "rosters" / "supplement" / "full_release_manifest.json"
    supplement = common.freeze_roster(
        supplement_rows,
        supplement_roster_path,
        dataset="Articraft-10K-GitHub-only-791",
        source_bindings=bindings,
    )

    old = load_roster_metadata(OLD_ROSTER)
    old_rows: list[dict[str, Any]] = []
    for original in old["rows"]:
        row = dict(original)
        record_id = str(row["asset_id"])
        row["category"] = str(categories[record_id])
        row["raw_category"] = str(categories[record_id])
        row["cohort_origin"] = "released_package_9996"
        old_rows.append(normalize_package_binding(row))
    merged_roster_path = output / "rosters" / "merged" / "full_release_manifest.json"
    merged = common.freeze_roster(
        [*old_rows, *supplement_rows],
        merged_roster_path,
        dataset="Articraft-10K-merged-reconstructed-10787",
        source_bindings=[
            {"name": "released_packages_9996", "path": str(RELEASE_ROOT.resolve())},
            *bindings,
        ],
    )
    if supplement["N_eval"] != EXPECTED_SUPPLEMENT:
        raise ValueError("supplement roster denominator mismatch")
    if merged["N_eval"] != EXPECTED_GITHUB:
        raise ValueError("merged roster denominator mismatch")
    if len({str(row["category"]) for row in merged["rows"]}) != EXPECTED_CATEGORIES:
        raise ValueError("merged category denominator mismatch")
    receipt = {
        "schema_version": "articraft_github_incremental_roster_receipt_v1",
        "created_at": utc_now(),
        "source_inventory": str((output / "cohort" / "source_inventory.json").resolve()),
        "supplement_roster": str(supplement_roster_path.resolve()),
        "supplement_roster_sha256": supplement["roster_sha256"],
        "supplement_manifest_content_sha256": supplement["manifest_content_sha256"],
        "supplement_n_eval": supplement["N_eval"],
        "supplement_j_eval": supplement["J_eval"],
        "merged_roster": str(merged_roster_path.resolve()),
        "merged_roster_sha256": merged["roster_sha256"],
        "merged_manifest_content_sha256": merged["manifest_content_sha256"],
        "merged_n_eval": merged["N_eval"],
        "merged_j_eval": merged["J_eval"],
        "category_count": EXPECTED_CATEGORIES,
    }
    receipt["receipt_content_sha256"] = canonical_sha256(receipt)
    atomic_json(output / "rosters" / "roster_receipt.json", receipt)
    return supplement, merged


def run_command(command: Sequence[str], *, cwd: Path = REPO_ROOT) -> None:
    print("running:", " ".join(command), flush=True)
    result = subprocess.run(command, cwd=cwd, env={**os.environ, **BLAS_ENVIRONMENT})
    if result.returncode:
        raise RuntimeError(f"command failed with exit {result.returncode}: {' '.join(command)}")


def evaluate_supplement(output: Path, *, workers: int) -> None:
    roster = output / "rosters" / "supplement" / "full_release_manifest.json"
    common.load_roster(roster, verify_sources=True)
    supplement = output / "supplement"
    commands = [
        [
            str(EVAL_PYTHON),
            str(EXP_ROOT / "scripts" / "run_table1_full_release.py"),
            "--roster",
            str(roster),
            "--output",
            str(supplement / "table1"),
            "--workers",
            str(max(1, workers)),
            "--timeout-seconds",
            "180",
            "--no-standard-parser",
            *(["--resume"] if (supplement / "table1" / "manifest.json").exists() else []),
        ],
        [
            str(EVAL_PYTHON),
            str(EXP_ROOT / "scripts" / "run_table2_full_release.py"),
            "--roster",
            str(roster),
            "--output",
            str(supplement / "table2"),
            "--workers",
            str(max(1, min(workers, 16))),
            "--timeout-seconds",
            "300",
            *(["--resume"] if (supplement / "table2" / "manifest.json").exists() else []),
        ],
        [
            str(EVAL_PYTHON),
            str(EXP_ROOT / "scripts" / "run_table3_full_release.py"),
            str(roster),
            str(supplement / "table3"),
            "--workers",
            str(max(1, workers)),
            "--timeout-seconds",
            "180",
        ],
    ]
    # The three static/FK tables are independent and read the same frozen inputs.
    with ThreadPoolExecutor(max_workers=3) as executor:
        futures = [executor.submit(run_command, command) for command in commands]
        for future in as_completed(futures):
            future.result()
    table4_command = [
        str(EVAL_PYTHON),
        str(EXP_ROOT / "scripts" / "run_table4_full_release.py"),
        "--dataset",
        "articraft",
        "--roster",
        str(roster),
        "--output",
        str(supplement / "table4"),
        "--workers",
        str(max(1, workers)),
        "--timeout-seconds",
        "300",
        "--sampling-protocol",
        "independent_sampling_v1",
        *(["--resume"] if (supplement / "table4" / "manifest.json").exists() else []),
    ]
    run_command(table4_command)


def bind_record(
    record: Mapping[str, Any],
    roster_row: Mapping[str, Any],
    *,
    identity_field: str,
) -> dict[str, Any]:
    result = dict(record)
    record_id = str(roster_row["asset_id"])
    result[identity_field] = record_id
    if identity_field == "asset_id" and "asset_key" in result:
        result["asset_key"] = record_id
    result["ordinal" if identity_field == "asset_id" else "order"] = int(roster_row["ordinal"])
    result["category"] = str(roster_row["category"])
    if identity_field == "asset_id" and "raw_category" in result:
        result["raw_category"] = str(roster_row["raw_category"])
    result["cohort_origin"] = str(roster_row["cohort_origin"])
    if "package_binding_sha256" in result:
        result["package_binding_sha256"] = roster_row.get("package_binding_sha256")
    return result


def merged_records(
    roster: Mapping[str, Any],
    old_path: Path,
    new_path: Path,
    *,
    identity_field: str = "asset_id",
) -> list[dict[str, Any]]:
    old_records = read_jsonl(old_path)
    new_records = read_jsonl(new_path)
    by_id: dict[str, dict[str, Any]] = {}
    for record in [*old_records, *new_records]:
        record_id = str(record.get(identity_field, ""))
        if not record_id or record_id in by_id:
            raise ValueError(f"duplicate or missing {identity_field}: {record_id!r}")
        by_id[record_id] = record
    rows = list(roster["rows"])
    expected_ids = {str(row["asset_id"]) for row in rows}
    if set(by_id) != expected_ids:
        raise ValueError(
            f"atomic record identity mismatch for {identity_field}: "
            f"records={len(by_id)} expected={len(expected_ids)}"
        )
    return [
        bind_record(by_id[str(row["asset_id"])], row, identity_field=identity_field)
        for row in rows
    ]


def add_summary_provenance(
    summary: dict[str, Any],
    *,
    table: str,
    records_path: Path,
    old_records_path: Path,
    new_records_path: Path,
    roster: Mapping[str, Any],
) -> dict[str, Any]:
    result = dict(summary)
    result.update(
        {
            "incremental_merge_schema_version": "articraft_9996_plus_791_atomic_merge_v1",
            "table": table,
            "cohort_composition": {
                "released_package_records": EXPECTED_RELEASED,
                "github_source_reconstructed_records": EXPECTED_SUPPLEMENT,
                "total": EXPECTED_GITHUB,
            },
            "records_sha256": sha256_file(records_path),
            "old_atomic_records": str(old_records_path.resolve()),
            "old_atomic_records_sha256": sha256_file(old_records_path),
            "supplement_atomic_records": str(new_records_path.resolve()),
            "supplement_atomic_records_sha256": sha256_file(new_records_path),
            "merged_roster_sha256": roster["roster_sha256"],
            "merged_roster_manifest_content_sha256": roster["manifest_content_sha256"],
            "completed_at": utc_now(),
        }
    )
    result.pop("summary_content_sha256", None)
    result["summary_content_sha256"] = canonical_sha256(result)
    return result


def merge_tables(output: Path) -> dict[str, Any]:
    roster_path = output / "rosters" / "merged" / "full_release_manifest.json"
    roster = load_roster_metadata(roster_path)
    rows_by_id = {str(row["asset_id"]): row for row in roster["rows"]}
    merged_root = output / "merged"
    paths = {
        "table1": (
            OLD_ROOT / "articraft" / "table1" / "asset_records.jsonl",
            output / "supplement" / "table1" / "asset_records.jsonl",
        ),
        "table2": (
            OLD_ROOT / "articraft" / "table2" / "asset_records.jsonl",
            output / "supplement" / "table2" / "asset_records.jsonl",
        ),
        "table3": (
            OLD_ROOT / "articraft" / "table3" / "records.jsonl",
            output / "supplement" / "table3" / "records.jsonl",
        ),
        "table4": (
            OLD_TABLE4 / "asset_records.jsonl",
            output / "supplement" / "table4" / "asset_records.jsonl",
        ),
    }
    modules = {
        "table1": load_module(EXP_ROOT / "scripts" / "run_table1_full_release.py", "articraft_incremental_table1"),
        "table2": load_module(EXP_ROOT / "scripts" / "run_table2_full_release.py", "articraft_incremental_table2"),
        "table3": load_module(EXP_ROOT / "scripts" / "run_table3_full_release.py", "articraft_incremental_table3"),
        "table4": load_module(EXP_ROOT / "scripts" / "run_table4_full_release.py", "articraft_incremental_table4"),
    }
    summaries: dict[str, dict[str, Any]] = {}
    for table in ("table1", "table2", "table3", "table4"):
        old_path, new_path = paths[table]
        identity = "dataset_id" if table == "table4" else "asset_id"
        records = merged_records(roster, old_path, new_path, identity_field=identity)
        table_dir = merged_root / table
        records_name = "records.jsonl" if table == "table3" else "asset_records.jsonl"
        records_path = table_dir / records_name
        atomic_jsonl(records_path, records)
        if table == "table1":
            summary = modules[table].aggregate_full_release(records, roster)
        elif table == "table2":
            summary = modules[table].aggregate_full_release(records, roster)
        elif table == "table3":
            summary = modules[table].aggregate_full_release(records, roster)
        else:
            summary = modules[table].aggregate_records(
                records, int(roster["N_eval"]), int(roster["J_eval"])
            )
            summary.update(
                {
                    "dataset": roster["dataset"],
                    "dataset_slug": "articraft",
                    "sampling_protocol": "independent_sampling_v1",
                    "roster_sha256": roster["roster_sha256"],
                    "roster_manifest_content_sha256": roster["manifest_content_sha256"],
                }
            )
        summary = add_summary_provenance(
            summary,
            table=table,
            records_path=records_path,
            old_records_path=old_path,
            new_records_path=new_path,
            roster=roster,
        )
        atomic_json(table_dir / "summary.json", summary)
        summaries[table] = summary

    old_states_path = OLD_TABLE4 / "state_records.jsonl"
    new_states_path = output / "supplement" / "table4" / "state_records.jsonl"
    states = [*read_jsonl(old_states_path), *read_jsonl(new_states_path)]
    for state in states:
        record_id = str(state.get("dataset_id", ""))
        row = rows_by_id.get(record_id)
        if row is None:
            raise ValueError(f"unbound Table 4 state record: {record_id}")
        state["order"] = int(row["ordinal"])
        state["category"] = str(row["category"])
        state["cohort_origin"] = str(row["cohort_origin"])
    phase_order = {"rest": 0, "single_joint_sweep": 1, "multi_joint_sobol": 2}
    states.sort(
        key=lambda row: (
            int(row["order"]),
            phase_order.get(str(row.get("phase")), 99),
            str(row.get("joint_name") or ""),
            int(row.get("sample_index", -1)),
        )
    )
    states_path = merged_root / "table4" / "state_records.jsonl"
    atomic_jsonl(states_path, states)
    table4_summary = summaries["table4"]
    table4_summary["state_records_sha256"] = sha256_file(states_path)
    table4_summary["old_state_records_sha256"] = sha256_file(old_states_path)
    table4_summary["supplement_state_records_sha256"] = sha256_file(new_states_path)
    table4_summary["state_records_count"] = len(states)
    table4_summary.pop("summary_content_sha256", None)
    table4_summary["summary_content_sha256"] = canonical_sha256(table4_summary)
    atomic_json(merged_root / "table4" / "summary.json", table4_summary)

    receipt = {
        "schema_version": "articraft_github_incremental_merged_receipt_v1",
        "created_at": utc_now(),
        "method": "reuse 9,996 released-package atomic records; evaluate 791 reconstructed GitHub-source packages; concatenate and re-aggregate",
        "n_eval": roster["N_eval"],
        "j_eval": roster["J_eval"],
        "category_count": len({str(row["category"]) for row in roster["rows"]}),
        "roster": str(roster_path.resolve()),
        "roster_sha256": roster["roster_sha256"],
        "roster_manifest_content_sha256": roster["manifest_content_sha256"],
        "source_inventory": str((output / "cohort" / "source_inventory.json").resolve()),
        "source_inventory_sha256": sha256_file(output / "cohort" / "source_inventory.json"),
        "tables": {
            table: {
                "summary": str((merged_root / table / "summary.json").resolve()),
                "summary_sha256": sha256_file(merged_root / table / "summary.json"),
                "summary_content_sha256": summary["summary_content_sha256"],
            }
            for table, summary in summaries.items()
        },
        "limitations": [
            "The 791 supplemental packages are reconstructed from frozen GitHub source records; they are not byte-identical members of the released_urdf archive.",
            "The old 9,996 atomic records remain bound to their original runner receipts; current aggregators are applied to the combined atomic records.",
        ],
    }
    receipt["receipt_content_sha256"] = canonical_sha256(receipt)
    atomic_json(output / "merged_receipt.json", receipt)
    return receipt


def verify(output: Path) -> dict[str, Any]:
    receipt = read_json(output / "merged_receipt.json")
    declared = receipt.pop("receipt_content_sha256", None)
    if declared != canonical_sha256(receipt):
        raise ValueError("merged receipt self-hash mismatch")
    receipt["receipt_content_sha256"] = declared
    roster = load_roster_metadata(
        output / "rosters" / "merged" / "full_release_manifest.json"
    )
    categories = {str(row["category"]) for row in roster["rows"]}
    origins: dict[str, int] = {}
    for row in roster["rows"]:
        origin = str(row.get("cohort_origin"))
        origins[origin] = origins.get(origin, 0) + 1
    if roster["N_eval"] != EXPECTED_GITHUB or len(categories) != EXPECTED_CATEGORIES:
        raise ValueError("merged roster denominator check failed")
    if origins != {
        "github_source_reconstructed_791": EXPECTED_SUPPLEMENT,
        "released_package_9996": EXPECTED_RELEASED,
    }:
        raise ValueError(f"cohort origin check failed: {origins}")
    if (
        int(receipt.get("n_eval", -1)) != int(roster["N_eval"])
        or int(receipt.get("j_eval", -1)) != int(roster["J_eval"])
        or int(receipt.get("category_count", -1)) != len(categories)
        or receipt.get("roster_sha256") != roster["roster_sha256"]
        or receipt.get("roster_manifest_content_sha256")
        != roster["manifest_content_sha256"]
    ):
        raise ValueError("merged receipt cohort binding mismatch")
    source_inventory_path = Path(str(receipt.get("source_inventory", "")))
    if (
        not source_inventory_path.is_file()
        or receipt.get("source_inventory_sha256")
        != sha256_file(source_inventory_path)
    ):
        raise ValueError("merged receipt source inventory binding mismatch")
    source_inventory = read_json(source_inventory_path)
    source_inventory_content_sha256 = source_inventory.pop(
        "inventory_content_sha256", None
    )
    if source_inventory_content_sha256 != canonical_sha256(source_inventory):
        raise ValueError("source inventory self-hash mismatch")
    compile_receipt_path = output / "compile" / "compile_receipt.json"
    compile_receipt = read_json(compile_receipt_path)
    compile_receipt_content_sha256 = compile_receipt.pop(
        "receipt_content_sha256", None
    )
    if compile_receipt_content_sha256 != canonical_sha256(compile_receipt):
        raise ValueError("compile receipt self-hash mismatch")
    compatibility_wrapper_path = Path(
        str(compile_receipt.get("compatibility_wrapper", ""))
    )
    compatibility_wrapper_sha256 = sha256_file(compatibility_wrapper_path)
    if (
        int(compile_receipt.get("strict_success_count", -1)) != 754
        or int(compile_receipt.get("fallback_cohort_count", -1)) != 37
        or int(compile_receipt.get("compiled_package_count", -1)) != 790
        or int(compile_receipt.get("retained_source_compile_failure_count", -1))
        != 1
        or int(compile_receipt.get("denominator_ready_count", -1))
        != EXPECTED_SUPPLEMENT
        or compile_receipt.get("compatibility_wrapper_sha256")
        != compatibility_wrapper_sha256
    ):
        raise ValueError("compile receipt count or compatibility-wrapper mismatch")
    input_checks = {
        "source_inventory_sha256": sha256_file(source_inventory_path),
        "source_inventory_content_sha256": source_inventory_content_sha256,
        "compile_receipt_sha256": sha256_file(compile_receipt_path),
        "compile_receipt_content_sha256": compile_receipt_content_sha256,
        "compatibility_wrapper_sha256": compatibility_wrapper_sha256,
        "strict_success_count": 754,
        "fallback_cohort_count": 37,
        "compiled_package_count": 790,
        "retained_source_compile_failure_count": 1,
        "denominator_ready_count": EXPECTED_SUPPLEMENT,
    }
    checks: dict[str, Any] = {}
    for table in ("table1", "table2", "table3", "table4"):
        summary_path = output / "merged" / table / "summary.json"
        summary = read_json(summary_path)
        summary_hash = summary.pop("summary_content_sha256", None)
        if summary_hash != canonical_sha256(summary):
            raise ValueError(f"{table} summary self-hash mismatch")
        summary["summary_content_sha256"] = summary_hash
        n_eval = int(summary.get("n_eval", summary.get("N_eval", -1)))
        j_eval = int(summary.get("j_eval", summary.get("J_eval", -1)))
        if n_eval != EXPECTED_GITHUB or j_eval != int(roster["J_eval"]):
            raise ValueError(f"{table} denominator mismatch")
        records_name = "records.jsonl" if table == "table3" else "asset_records.jsonl"
        records_path = output / "merged" / table / records_name
        line_count = sum(1 for line in records_path.open(encoding="utf-8") if line.strip())
        if line_count != EXPECTED_GITHUB:
            raise ValueError(f"{table} atomic record count mismatch")
        records_sha256 = sha256_file(records_path)
        if summary.get("records_sha256") != records_sha256:
            raise ValueError(f"{table} merged records hash mismatch")
        old_records_path = Path(str(summary.get("old_atomic_records", "")))
        supplement_records_path = Path(str(summary.get("supplement_atomic_records", "")))
        old_records_sha256 = sha256_file(old_records_path)
        supplement_records_sha256 = sha256_file(supplement_records_path)
        if summary.get("old_atomic_records_sha256") != old_records_sha256:
            raise ValueError(f"{table} old atomic records hash mismatch")
        if summary.get("supplement_atomic_records_sha256") != supplement_records_sha256:
            raise ValueError(f"{table} supplement atomic records hash mismatch")
        receipt_binding = receipt.get("tables", {}).get(table, {})
        if (
            Path(str(receipt_binding.get("summary", ""))).resolve()
            != summary_path.resolve()
            or receipt_binding.get("summary_sha256") != sha256_file(summary_path)
            or receipt_binding.get("summary_content_sha256") != summary_hash
        ):
            raise ValueError(f"{table} merged receipt summary binding mismatch")
        checks[table] = {
            "n_eval": n_eval,
            "j_eval": j_eval,
            "records": line_count,
            "records_sha256": records_sha256,
            "old_atomic_records_sha256": old_records_sha256,
            "supplement_atomic_records_sha256": supplement_records_sha256,
            "summary_sha256": sha256_file(summary_path),
        }
        if table == "table4":
            state_records_path = output / "merged" / "table4" / "state_records.jsonl"
            state_records_count = sum(
                1 for line in state_records_path.open(encoding="utf-8") if line.strip()
            )
            state_records_sha256 = sha256_file(state_records_path)
            old_state_records_sha256 = sha256_file(OLD_TABLE4 / "state_records.jsonl")
            supplement_state_records_sha256 = sha256_file(
                output / "supplement" / "table4" / "state_records.jsonl"
            )
            if (
                state_records_count != int(summary.get("state_records_count", -1))
                or state_records_count != int(summary.get("state_records_executed", -1))
                or state_records_sha256 != summary.get("state_records_sha256")
                or old_state_records_sha256
                != summary.get("old_state_records_sha256")
                or supplement_state_records_sha256
                != summary.get("supplement_state_records_sha256")
            ):
                raise ValueError("table4 state records binding mismatch")
            checks[table].update(
                {
                    "state_records": state_records_count,
                    "state_records_sha256": state_records_sha256,
                    "old_state_records_sha256": old_state_records_sha256,
                    "supplement_state_records_sha256": supplement_state_records_sha256,
                }
            )
    verification_core = {
        "schema_version": "articraft_github_incremental_verification_v1",
        "status": "PASS",
        "n_eval": roster["N_eval"],
        "j_eval": roster["J_eval"],
        "category_count": len(categories),
        "cohort_origins": origins,
        "inputs": input_checks,
        "checks": checks,
    }
    verification_path = output / "verification.json"
    verified_at = utc_now()
    if verification_path.exists():
        existing = read_json(verification_path)
        existing_hash = existing.pop("verification_content_sha256", None)
        if existing_hash != canonical_sha256(existing):
            raise ValueError("existing verification self-hash mismatch")
        existing_verified_at = existing.pop("verified_at", None)
        if existing == verification_core and isinstance(existing_verified_at, str):
            verified_at = existing_verified_at
    verification = dict(verification_core)
    verification["verified_at"] = verified_at
    verification["verification_content_sha256"] = canonical_sha256(verification)
    atomic_json(verification_path, verification)
    return verification


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "phase",
        choices=(
            "prepare",
            "compile",
            "fallback",
            "freeze",
            "evaluate",
            "merge",
            "verify",
            "all",
        ),
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=32)
    parser.add_argument("--compile-timeout-seconds", type=float, default=900.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    phases = (
        ("prepare", lambda: source_inventory(output, create_staging=True)),
        (
            "compile",
            lambda: run_compile(
                output,
                workers=args.workers,
                timeout_seconds=args.compile_timeout_seconds,
            ),
        ),
        (
            "fallback",
            lambda: run_compile_fallback(
                output,
                workers=args.workers,
                timeout_seconds=args.compile_timeout_seconds,
            ),
        ),
        ("freeze", lambda: build_rosters(output, workers=args.workers)),
        ("evaluate", lambda: evaluate_supplement(output, workers=args.workers)),
        ("merge", lambda: merge_tables(output)),
        ("verify", lambda: verify(output)),
    )
    for name, action in phases:
        if args.phase not in {name, "all"}:
            continue
        print(f"[{name}] start", flush=True)
        action()
        print(f"[{name}] complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
