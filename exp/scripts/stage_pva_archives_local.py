#!/usr/bin/env python3
"""Safely stage the frozen PV-A archive roster into a local extracted mirror."""

from __future__ import annotations

import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import sqlite3
import stat
import subprocess
import tarfile
import tempfile
from typing import Any, Iterable, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
DEFAULT_SOURCE_EVALUATION = (
    SCRIPT.parents[1] / "runtime" / "pva_table1234_full_release_20260826" / "evaluation"
)
DEFAULT_ARCHIVES_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A/archives")
MANIFEST_SCHEMA = "pva_local_archive_stage_manifest_v1"
MARKER_SCHEMA = "pva_local_archive_stage_marker_v1"
SUMMARY_SCHEMA = "pva_local_archive_stage_summary_v1"
ARTIFACT_SCHEMA = "pva_local_archive_stage_artifact_manifest_v1"
SOURCE_RECEIPT_SCHEMA = "pva_table1234_full_release_receipt_v1"
SOURCE_DB_SCHEMA = "pva_table1234_results_db_v1"
HEX_DIGITS = frozenset("0123456789abcdef")
# The first formal staging attempt used this exact source revision.  Its only
# incompatible behavior was closing zstd's stdout at the tar end marker.
RESUME_COMPATIBLE_SOURCE_CODE_SHA256 = frozenset(
    {"fd159a207b97b521c4630a60c62d2c89e62c9dea0dc21b04d461cd8c40d4f169"}
)


@dataclass(frozen=True)
class ArchivePlan:
    archive_name: str
    category: str
    expected_sha256: str
    source_asset_count: int
    archive_path: Path


@dataclass(frozen=True)
class ArchiveAudit:
    members: tuple[tuple[str, str, int], ...]
    tar_member_count: int
    regular_file_count: int
    directory_count: int
    total_regular_bytes: int
    top_level_entries: tuple[str, ...]
    member_manifest_sha256: str


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
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _file_identity(path: Path) -> tuple[int, int, int, int]:
    details = path.stat()
    return (details.st_dev, details.st_ino, details.st_size, details.st_mtime_ns)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _atomic_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, raw_temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(raw_temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, value: Any) -> None:
    _atomic_bytes(path, (_canonical_text(value) + "\n").encode("utf-8"))


def _load_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _valid_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and set(value).issubset(HEX_DIGITS)
    )


def _safe_component(value: str, label: str) -> str:
    if (
        not value
        or value in {".", ".."}
        or "/" in value
        or "\\" in value
        or "\x00" in value
        or len(os.fsencode(value)) > 255
    ):
        raise ValueError(f"unsafe {label}: {value!r}")
    return value


def _safe_archive_name(value: str) -> str:
    _safe_component(value, "archive name")
    if not value.endswith(".tar.zst"):
        raise ValueError(f"unexpected archive suffix: {value!r}")
    return value


def _require_plain_directory(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_dir():
        raise ValueError(f"{label} must be a non-symlink directory: {path}")
    return path.resolve(strict=True)


def _require_plain_file(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a non-symlink regular file: {path}")
    return path.resolve(strict=True)


def _normalize_member_name(raw_name: str) -> str:
    if not raw_name or "\\" in raw_name or "\x00" in raw_name:
        raise ValueError(f"unsafe tar member path: {raw_name!r}")
    if raw_name.startswith("/"):
        raise ValueError(f"absolute tar member path: {raw_name!r}")
    raw_parts = raw_name.rstrip("/").split("/")
    while raw_parts and raw_parts[0] == ".":
        raw_parts.pop(0)
    if not raw_parts or any(part in {"", ".", ".."} for part in raw_parts):
        raise ValueError(f"unsafe tar member path: {raw_name!r}")
    if any(len(os.fsencode(part)) > 255 for part in raw_parts):
        raise ValueError(f"oversized tar member path component: {raw_name!r}")
    normalized = PurePosixPath(*raw_parts).as_posix()
    if len(os.fsencode(normalized)) > 4095:
        raise ValueError(f"oversized tar member path: {raw_name!r}")
    return normalized


def _read_source_mapping(
    source_evaluation: Path, archives_root: Path
) -> tuple[list[ArchivePlan], dict[str, Any]]:
    source_evaluation = _require_plain_directory(source_evaluation, "source evaluation")
    archives_root = _require_plain_directory(archives_root, "archive root")
    receipt_path = _require_plain_file(
        source_evaluation / "full_release_receipt.json", "source receipt"
    )
    database_path = _require_plain_file(
        source_evaluation / "results.sqlite3", "source database"
    )
    receipt = _load_json_object(receipt_path)
    if receipt.get("schema_version") != SOURCE_RECEIPT_SCHEMA:
        raise ValueError("source receipt schema mismatch")
    if receipt.get("receipt_content_sha256") != _self_hash(
        receipt, "receipt_content_sha256"
    ):
        raise ValueError("source receipt self-hash mismatch")
    if not _valid_sha256(receipt.get("result_database_sha256")):
        raise ValueError("source receipt has no valid database SHA256")

    connection = sqlite3.connect(f"{database_path.as_uri()}?mode=ro", uri=True)
    connection.execute("PRAGMA query_only=ON")
    by_name: dict[str, dict[str, Any]] = {}
    total_rows = 0
    categories: set[str] = set()
    try:
        meta = {
            str(key): json.loads(value)
            for key, value in connection.execute("SELECT key, value FROM meta")
        }
        if meta.get("schema_version") != SOURCE_DB_SCHEMA:
            raise ValueError("source database schema mismatch")
        if meta.get("asset_import_state") != "COMPLETE":
            raise ValueError("source database asset import is not complete")
        query = (
            "SELECT ordinal, asset_id, category, row_sha256, row_json "
            "FROM assets ORDER BY ordinal"
        )
        for expected_ordinal, values in enumerate(connection.execute(query)):
            ordinal, asset_id, category, row_sha256, row_text = values
            if int(ordinal) != expected_ordinal:
                raise ValueError("source database asset ordinals are not contiguous")
            row = json.loads(row_text)
            if not isinstance(row, dict):
                raise ValueError(f"source row {ordinal} is not an object")
            if hashlib.sha256(row_text.encode("utf-8")).hexdigest() != row_sha256:
                raise ValueError(f"source row hash mismatch at ordinal {ordinal}")
            frozen_category = str(row.get("raw_category", row.get("category", "")))
            if (
                str(row.get("asset_id", "")) != str(asset_id)
                or frozen_category != str(category)
            ):
                raise ValueError(f"source row identity mismatch at ordinal {ordinal}")
            archive_name = _safe_archive_name(str(row.get("archive_name", "")))
            frozen_category = _safe_component(frozen_category, "category")
            archive_sha256 = row.get("archive_sha256")
            if not _valid_sha256(archive_sha256):
                raise ValueError(f"invalid archive SHA256 at ordinal {ordinal}")
            binding = by_name.setdefault(
                archive_name,
                {
                    "category": frozen_category,
                    "expected_sha256": archive_sha256,
                    "source_asset_count": 0,
                },
            )
            if (
                binding["category"] != frozen_category
                or binding["expected_sha256"] != archive_sha256
            ):
                raise ValueError(f"conflicting source binding for {archive_name}")
            binding["source_asset_count"] += 1
            total_rows += 1
            categories.add(frozen_category)
    finally:
        connection.close()

    if total_rows != int(receipt.get("N_eval", -1)):
        raise ValueError("source row count does not match receipt")
    if len(categories) != int(receipt.get("eval_category_count", -1)):
        raise ValueError("source category count does not match receipt")

    actual_names: set[str] = set()
    for candidate in archives_root.iterdir():
        if candidate.name.endswith(".tar.zst"):
            _safe_archive_name(candidate.name)
            _require_plain_file(candidate, "PV-A archive")
            actual_names.add(candidate.name)
    expected_names = set(by_name)
    if actual_names != expected_names:
        missing = sorted(expected_names - actual_names)[:5]
        extra = sorted(actual_names - expected_names)[:5]
        raise ValueError(f"archive root closure mismatch: missing={missing}, extra={extra}")

    plans = [
        ArchivePlan(
            archive_name=name,
            category=str(binding["category"]),
            expected_sha256=str(binding["expected_sha256"]),
            source_asset_count=int(binding["source_asset_count"]),
            archive_path=(archives_root / name).resolve(strict=True),
        )
        for name, binding in sorted(by_name.items())
    ]
    source = {
        "source_evaluation": str(source_evaluation),
        "source_receipt": str(receipt_path),
        "source_receipt_sha256": _sha256_file(receipt_path),
        "source_receipt_content_sha256": receipt["receipt_content_sha256"],
        "source_database": str(database_path),
        "source_database_declared_sha256": receipt["result_database_sha256"],
        "source_database_bytes": database_path.stat().st_size,
        "source_asset_count": total_rows,
        "source_category_count": len(categories),
        "archives_root": str(archives_root),
    }
    return plans, source


def _execution_manifest(plans: Sequence[ArchivePlan], source: Mapping[str, Any]) -> dict[str, Any]:
    archive_rows = [
        {
            "archive_name": plan.archive_name,
            "category": plan.category,
            "expected_sha256": plan.expected_sha256,
            "source_asset_count": plan.source_asset_count,
        }
        for plan in plans
    ]
    value: dict[str, Any] = {
        "schema_version": MANIFEST_SCHEMA,
        "source": dict(source),
        "archive_count": len(plans),
        "category_count": len({plan.category for plan in plans}),
        "source_asset_count": sum(plan.source_asset_count for plan in plans),
        "archive_mapping": archive_rows,
        "archive_mapping_content_sha256": _canonical_sha256(archive_rows),
        "extraction_policy": {
            "member_types": ["directory", "regular_file"],
            "member_paths": "safe_relative_posix_paths_only",
            "archive_validation_before_extraction": True,
            "category_groups_are_serial": True,
            "ownership_and_permissions_preserved": False,
        },
        "source_code_sha256": _sha256_file(SCRIPT),
    }
    value["manifest_content_sha256"] = _self_hash(value, "manifest_content_sha256")
    return value


def _resume_manifest_compatible(
    observed: Mapping[str, Any], requested: Mapping[str, Any]
) -> bool:
    if observed.get("manifest_content_sha256") != _self_hash(
        observed, "manifest_content_sha256"
    ):
        return False
    if observed.get("source_code_sha256") not in RESUME_COMPATIBLE_SOURCE_CODE_SHA256:
        return False
    observed_contract = dict(observed)
    requested_contract = dict(requested)
    for contract in (observed_contract, requested_contract):
        contract.pop("source_code_sha256", None)
        contract.pop("manifest_content_sha256", None)
    return observed_contract == requested_contract


def _audit_archive(archive_path: Path) -> ArchiveAudit:
    zstd = shutil.which("zstd")
    if zstd is None:
        raise RuntimeError("zstd executable is required")
    process = subprocess.Popen(
        [zstd, "-q", "-d", "-c", "--", str(archive_path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert process.stdout is not None
    declared: dict[str, tuple[str, int]] = {}
    tar_member_count = 0
    try:
        with tarfile.open(fileobj=process.stdout, mode="r|") as archive:
            for member in archive:
                tar_member_count += 1
                name = _normalize_member_name(member.name)
                if member.isfile():
                    kind, size = "file", int(member.size)
                elif member.isdir():
                    kind, size = "directory", 0
                else:
                    raise ValueError(
                        f"unsupported tar member type in {archive_path.name}: {member.name!r}"
                    )
                if name in declared:
                    raise ValueError(
                        f"duplicate tar member in {archive_path.name}: {name!r}"
                    )
                declared[name] = (kind, size)
        # tarfile stops at the first pair of zero end blocks.  Consume record
        # padding so zstd can finish normally instead of receiving SIGPIPE.
        for _ in iter(lambda: process.stdout.read(1024 * 1024), b""):
            pass
    except BaseException:
        process.kill()
        process.wait()
        raise
    finally:
        process.stdout.close()
    assert process.stderr is not None
    stderr = process.stderr.read().decode("utf-8", errors="replace")
    return_code = process.wait()
    if return_code != 0:
        raise ValueError(f"zstd validation failed for {archive_path.name}: {stderr.strip()}")
    if not declared:
        raise ValueError(f"empty archive: {archive_path.name}")

    closure = dict(declared)
    for name in tuple(declared):
        parts = PurePosixPath(name).parts
        for length in range(1, len(parts)):
            ancestor = PurePosixPath(*parts[:length]).as_posix()
            existing = closure.get(ancestor)
            if existing is not None and existing[0] != "directory":
                raise ValueError(
                    f"file is an ancestor of another member in {archive_path.name}: {ancestor}"
                )
            closure.setdefault(ancestor, ("directory", 0))
    members = tuple(
        (name, kind, size)
        for name, (kind, size) in sorted(closure.items())
    )
    member_rows = [
        {"path": name, "type": kind, "size": size}
        for name, kind, size in members
    ]
    return ArchiveAudit(
        members=members,
        tar_member_count=tar_member_count,
        regular_file_count=sum(kind == "file" for _, kind, _ in members),
        directory_count=sum(kind == "directory" for _, kind, _ in members),
        total_regular_bytes=sum(size for _, kind, size in members if kind == "file"),
        top_level_entries=tuple(sorted({PurePosixPath(name).parts[0] for name in closure})),
        member_manifest_sha256=_canonical_sha256(member_rows),
    )


def _marker_stable_fields(
    plan: ArchivePlan,
    manifest_content_sha256: str,
    audit: ArchiveAudit,
    archive_size: int,
    observed_sha256: str,
) -> dict[str, Any]:
    return {
        "schema_version": MARKER_SCHEMA,
        "manifest_content_sha256": manifest_content_sha256,
        "archive_name": plan.archive_name,
        "category": plan.category,
        "source_asset_count": plan.source_asset_count,
        "archive_size": archive_size,
        "expected_archive_sha256": plan.expected_sha256,
        "observed_archive_sha256": observed_sha256,
        "tar_member_count": audit.tar_member_count,
        "regular_file_count": audit.regular_file_count,
        "directory_count": audit.directory_count,
        "total_regular_bytes": audit.total_regular_bytes,
        "top_level_entries": list(audit.top_level_entries),
        "member_manifest_sha256": audit.member_manifest_sha256,
    }


def _verify_marker(
    path: Path, expected_status: str, stable_fields: Mapping[str, Any]
) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"invalid stage marker: {path}")
    value = _load_json_object(path)
    if value.get("marker_content_sha256") != _self_hash(
        value, "marker_content_sha256"
    ):
        raise ValueError(f"stage marker self-hash mismatch: {path}")
    if value.get("status") != expected_status:
        raise ValueError(f"stage marker status mismatch: {path}")
    for key, expected in stable_fields.items():
        if value.get(key) != expected:
            raise ValueError(f"stage marker binding mismatch for {key}: {path}")
    return value


def _assert_existing_tree_safe(path: Path) -> None:
    mode = path.lstat().st_mode
    if stat.S_ISLNK(mode) or not (stat.S_ISDIR(mode) or stat.S_ISREG(mode)):
        raise ValueError(f"unsafe existing extraction path: {path}")
    if stat.S_ISREG(mode):
        return
    for root, directories, files in os.walk(path, followlinks=False):
        root_path = Path(root)
        for name in directories + files:
            candidate = root_path / name
            candidate_mode = candidate.lstat().st_mode
            if stat.S_ISLNK(candidate_mode) or not (
                stat.S_ISDIR(candidate_mode) or stat.S_ISREG(candidate_mode)
            ):
                raise ValueError(f"unsafe existing extraction path: {candidate}")


def _prepare_category_target(
    category_dir: Path, audit: ArchiveAudit, *, recovering: bool
) -> None:
    category_dir.mkdir(parents=True, exist_ok=True)
    if category_dir.is_symlink() or not category_dir.is_dir():
        raise ValueError(f"category output is not a plain directory: {category_dir}")
    for name in audit.top_level_entries:
        target = category_dir / name
        if os.path.lexists(target):
            if not recovering:
                raise ValueError(f"archive would overwrite existing output: {target}")
            _assert_existing_tree_safe(target)


def _extract_archive(archive_path: Path, category_dir: Path) -> None:
    tar = shutil.which("tar")
    if tar is None:
        raise RuntimeError("GNU tar executable is required")
    subprocess.run(
        [
            tar,
            "--extract",
            "--zstd",
            "--file",
            str(archive_path),
            "--directory",
            str(category_dir),
            "--no-same-owner",
            "--no-same-permissions",
            "--no-overwrite-dir",
            "--delay-directory-restore",
        ],
        check=True,
    )


def _verify_extracted(category_dir: Path, audit: ArchiveAudit) -> None:
    expected = {name: (kind, size) for name, kind, size in audit.members}
    actual: set[str] = set()
    for top_name in audit.top_level_entries:
        top = category_dir / top_name
        if not os.path.lexists(top):
            raise ValueError(f"missing extracted top-level path: {top}")
        candidates: Iterable[Path]
        if top.is_dir() and not top.is_symlink():
            candidates = [top, *sorted(top.rglob("*"))]
        else:
            candidates = [top]
        for candidate in candidates:
            relative = candidate.relative_to(category_dir).as_posix()
            mode = candidate.lstat().st_mode
            if stat.S_ISLNK(mode):
                raise ValueError(f"extracted symlink is forbidden: {candidate}")
            if stat.S_ISDIR(mode):
                observed = ("directory", 0)
            elif stat.S_ISREG(mode):
                observed = ("file", candidate.stat().st_size)
            else:
                raise ValueError(f"extracted special file is forbidden: {candidate}")
            if expected.get(relative) != observed:
                raise ValueError(f"extracted member mismatch: {candidate}")
            actual.add(relative)
    if actual != set(expected):
        missing = sorted(set(expected) - actual)[:5]
        extra = sorted(actual - set(expected))[:5]
        raise ValueError(f"extracted member closure mismatch: missing={missing}, extra={extra}")


def _stage_archive(
    plan: ArchivePlan, output: Path, manifest_content_sha256: str
) -> dict[str, Any]:
    archive = _require_plain_file(plan.archive_path, "PV-A archive")
    archive_identity = _file_identity(archive)
    archive_size = archive_identity[2]
    observed_sha256 = _sha256_file(archive)
    if _file_identity(archive) != archive_identity:
        raise ValueError(f"archive changed while hashing: {plan.archive_name}")
    if observed_sha256 != plan.expected_sha256:
        raise ValueError(f"archive SHA256 mismatch: {plan.archive_name}")
    audit = _audit_archive(archive)
    if _file_identity(archive) != archive_identity:
        raise ValueError(f"archive changed while validating: {plan.archive_name}")
    stable = _marker_stable_fields(
        plan, manifest_content_sha256, audit, archive_size, observed_sha256
    )
    marker_dir = output / "markers"
    done_path = marker_dir / f"{plan.archive_name}.done.json"
    inprogress_path = marker_dir / f"{plan.archive_name}.inprogress.json"
    category_dir = output / "extracted" / plan.category

    if done_path.exists() or done_path.is_symlink():
        done = _verify_marker(done_path, "COMPLETE", stable)
        _verify_extracted(category_dir, audit)
        inprogress_path.unlink(missing_ok=True)
        return done

    recovering = inprogress_path.exists() or inprogress_path.is_symlink()
    if recovering:
        _verify_marker(inprogress_path, "IN_PROGRESS", stable)
    else:
        inprogress = {**stable, "status": "IN_PROGRESS", "started_at_utc": _utc_now()}
        inprogress["marker_content_sha256"] = _self_hash(
            inprogress, "marker_content_sha256"
        )
        _atomic_json(inprogress_path, inprogress)

    _prepare_category_target(category_dir, audit, recovering=recovering)
    _extract_archive(archive, category_dir)
    if _file_identity(archive) != archive_identity:
        raise ValueError(f"archive changed during extraction: {plan.archive_name}")
    _verify_extracted(category_dir, audit)
    done = {**stable, "status": "COMPLETE", "completed_at_utc": _utc_now()}
    done["marker_content_sha256"] = _self_hash(done, "marker_content_sha256")
    _atomic_json(done_path, done)
    inprogress_path.unlink(missing_ok=True)
    return done


def _stage_category(
    plans: Sequence[ArchivePlan], output: Path, manifest_content_sha256: str
) -> list[dict[str, Any]]:
    records = []
    for plan in sorted(plans, key=lambda item: item.archive_name):
        records.append(_stage_archive(plan, output, manifest_content_sha256))
    return records


def _write_final_closure(
    output: Path, manifest: Mapping[str, Any], records: Sequence[Mapping[str, Any]]
) -> Path:
    ordered = sorted(records, key=lambda row: str(row["archive_name"]))
    records_path = output / "archive_records.jsonl"
    _atomic_bytes(
        records_path,
        b"".join((_canonical_text(row) + "\n").encode("utf-8") for row in ordered),
    )
    summary = {
        "schema_version": SUMMARY_SCHEMA,
        "status": "COMPLETE",
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "archive_count": len(ordered),
        "category_count": len({str(row["category"]) for row in ordered}),
        "source_asset_count": sum(int(row["source_asset_count"]) for row in ordered),
        "archive_bytes": sum(int(row["archive_size"]) for row in ordered),
        "regular_file_count": sum(int(row["regular_file_count"]) for row in ordered),
        "extracted_regular_bytes": sum(int(row["total_regular_bytes"]) for row in ordered),
        "archive_records_sha256": _sha256_file(records_path),
        "marker_set_content_sha256": _canonical_sha256(
            [row["marker_content_sha256"] for row in ordered]
        ),
        "extracted_root": "extracted",
    }
    summary["summary_content_sha256"] = _self_hash(summary, "summary_content_sha256")
    summary_path = output / "summary.json"
    _atomic_json(summary_path, summary)

    names = ["manifest.json", "archive_records.jsonl", "summary.json"] + [
        f"markers/{row['archive_name']}.done.json" for row in ordered
    ]
    files = []
    for name in sorted(names):
        path = output / name
        files.append({"path": name, "bytes": path.stat().st_size, "sha256": _sha256_file(path)})
    artifact: dict[str, Any] = {
        "schema_version": ARTIFACT_SCHEMA,
        "files": files,
    }
    artifact["artifact_manifest_content_sha256"] = _self_hash(
        artifact, "artifact_manifest_content_sha256"
    )
    artifact_path = output / "artifact_manifest.json"
    _atomic_json(artifact_path, artifact)
    return artifact_path


def stage_pva_archives(
    source_evaluation: Path,
    archives_root: Path,
    output: Path,
    *,
    workers: int = 8,
    resume: bool = False,
) -> Path:
    if workers <= 0:
        raise ValueError("workers must be positive")
    source_evaluation = Path(source_evaluation)
    archives_root = Path(archives_root)
    output_input = Path(output)
    if output_input.is_symlink():
        raise ValueError(f"output must not be a symlink: {output_input}")
    output = output_input.resolve(strict=False)
    archive_root_resolved = archives_root.resolve(strict=True)
    if output == archive_root_resolved or archive_root_resolved in output.parents:
        raise ValueError("output must not be inside the archive root")
    if output.exists() and not output.is_dir():
        raise ValueError(f"output is not a directory: {output}")
    if output.exists() and any(output.iterdir()) and not resume:
        raise ValueError(f"output is not empty; use --resume: {output}")
    output.mkdir(parents=True, exist_ok=True)

    lock_path = output / ".stage.lock"
    with lock_path.open("a+b") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError(f"another staging process owns {output}") from error

        plans, source = _read_source_mapping(source_evaluation, archives_root)
        requested_manifest = _execution_manifest(plans, source)
        active_manifest = requested_manifest
        manifest_path = output / "manifest.json"
        if manifest_path.exists() or manifest_path.is_symlink():
            if not resume:
                raise ValueError(f"manifest already exists; use --resume: {manifest_path}")
            observed_manifest = _load_json_object(manifest_path)
            if observed_manifest != requested_manifest:
                if not _resume_manifest_compatible(
                    observed_manifest, requested_manifest
                ):
                    raise ValueError("resume manifest binding mismatch")
                active_manifest = observed_manifest
        else:
            _atomic_json(manifest_path, requested_manifest)
        manifest_sha = str(active_manifest["manifest_content_sha256"])

        groups: dict[str, list[ArchivePlan]] = defaultdict(list)
        for plan in plans:
            groups[plan.category].append(plan)
        records: list[dict[str, Any]] = []
        with ThreadPoolExecutor(
            max_workers=min(workers, max(1, len(groups))),
            thread_name_prefix="pva-stage",
        ) as executor:
            futures = {
                executor.submit(_stage_category, group, output, manifest_sha): category
                for category, group in sorted(groups.items())
            }
            for future in as_completed(futures):
                records.extend(future.result())
        if len(records) != len(plans):
            raise ValueError("staging result count mismatch")
        return _write_final_closure(output, active_manifest, records)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-evaluation", type=Path, default=DEFAULT_SOURCE_EVALUATION)
    parser.add_argument("--archives-root", type=Path, default=DEFAULT_ARCHIVES_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--resume", action="store_true")
    return parser.parse_args()


def main() -> int:
    arguments = _parse_args()
    artifact = stage_pva_archives(
        arguments.source_evaluation,
        arguments.archives_root,
        arguments.output,
        workers=arguments.workers,
        resume=arguments.resume,
    )
    print(artifact)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
