#!/usr/bin/env python3
"""Atomically publish a sealed PV-A mimic-aware Table 4 full release."""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
if str(SCRIPT.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT.parent))

import check_pva_table4_mimic_aware_full_release as checker

CHECKER_SOURCE = Path(checker.__file__).resolve()
ARTIFACT_SCHEMA = "pva_table4_mimic_aware_artifacts_v1"
RECEIPT_SCHEMA = "pva_table4_mimic_aware_full_release_receipt_v1"
MANIFEST_SCHEMA = "pva_table4_mimic_aware_full_release_run_v2"
CHECKPOINT_SCHEMA = "pva_table4_mimic_aware_checkpoint_v1"
PROGRESS_SCHEMA = "pva_table4_mimic_aware_progress_v1"
COPY_RECEIPT_SCHEMA = "pva_table4_mimic_aware_atomic_copy_receipt_v1"
SAMPLING_PROTOCOL = "mimic_aware_independent_sampling_v2"
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v2"
SEALED_ARTIFACTS = (
    "manifest.json",
    "protocol_snapshot.md",
    "records.jsonl",
    "asset_records.jsonl",
    "state_records.jsonl",
    "summary.json",
    "summary.md",
    "checkpoint.json",
    "results.sqlite3",
)
PUBLISHED_FILES = SEALED_ARTIFACTS + (
    "artifact_manifest.json",
    "full_release_receipt.json",
    "progress.json",
)
INTERNAL_COPY_RECEIPT = "publication_copy_receipt.json"


class PublicationError(ValueError):
    pass


def _canonical_text(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    payload = dict(value)
    payload.pop(field, None)
    return hashlib.sha256(_canonical_text(payload).encode("utf-8")).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_file(path: Path, label: str) -> Path:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError as error:
        raise PublicationError(f"missing {label}: {path}") from error
    if stat.S_ISLNK(mode) or not stat.S_ISREG(mode):
        raise PublicationError(f"{label} is not a regular non-symlink file: {path}")
    return path


def _json(path: Path, label: str) -> dict[str, Any]:
    _regular_file(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise PublicationError(f"invalid JSON in {label}: {path}") from error
    if not isinstance(value, dict):
        raise PublicationError(f"{label} must be a JSON object")
    return value


def _require_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != _self_hash(value, field):
        raise PublicationError(f"{label} self-hash mismatch")


def _relative_file(root: Path, value: Any, expected: str, label: str) -> Path:
    if value != expected:
        raise PublicationError(f"{label} path binding mismatch")
    return _regular_file(root / expected, label)


def verify_source(source: Path) -> dict[str, Any]:
    source = Path(source)
    try:
        source_mode = source.lstat().st_mode
    except FileNotFoundError as error:
        raise PublicationError(f"source does not exist: {source}") from error
    if stat.S_ISLNK(source_mode) or not stat.S_ISDIR(source_mode):
        raise PublicationError("source must be a non-symlink directory")

    progress = _json(source / "progress.json", "progress")
    if progress.get("schema_version") != PROGRESS_SCHEMA or progress.get("status") != "COMPLETE":
        raise PublicationError("progress is not COMPLETE with the expected schema")

    checkpoint = _json(source / "checkpoint.json", "checkpoint")
    if checkpoint.get("schema_version") != CHECKPOINT_SCHEMA or checkpoint.get("state") != "complete":
        raise PublicationError("checkpoint is not complete with the expected schema")
    _require_self_hash(checkpoint, "checkpoint_content_sha256", "checkpoint")

    manifest = _json(source / "manifest.json", "manifest")
    if manifest.get("schema_version") != MANIFEST_SCHEMA:
        raise PublicationError("manifest schema mismatch")
    _require_self_hash(manifest, "manifest_content_sha256", "manifest")
    if manifest.get("classification") != "FORMAL_FULL_RELEASE" or manifest.get("limit") is not None:
        raise PublicationError("manifest is not an unlimited FORMAL_FULL_RELEASE")
    if manifest.get("sampling_protocol") != SAMPLING_PROTOCOL or manifest.get("protocol_id") != PROTOCOL_ID:
        raise PublicationError("manifest protocol mismatch")

    artifact = _json(source / "artifact_manifest.json", "artifact manifest")
    if artifact.get("schema_version") != ARTIFACT_SCHEMA:
        raise PublicationError("artifact manifest schema mismatch")
    _require_self_hash(artifact, "artifact_manifest_content_sha256", "artifact manifest")
    entries = artifact.get("artifacts")
    if not isinstance(entries, list) or len(entries) != len(SEALED_ARTIFACTS):
        raise PublicationError("artifact manifest must contain exactly nine entries")
    by_path: dict[str, Mapping[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping) or set(entry) != {"path", "bytes", "sha256"}:
            raise PublicationError("artifact manifest entry has an invalid field set")
        name = entry.get("path")
        if not isinstance(name, str) or name in by_path:
            raise PublicationError("artifact manifest path is invalid or duplicated")
        by_path[name] = entry
    if set(by_path) != set(SEALED_ARTIFACTS):
        raise PublicationError("artifact manifest does not name the exact sealed artifact set")
    for name in SEALED_ARTIFACTS:
        path = _regular_file(source / name, f"artifact {name}")
        entry = by_path[name]
        if type(entry.get("bytes")) is not int or entry["bytes"] < 0 or path.stat().st_size != entry["bytes"]:
            raise PublicationError(f"artifact size mismatch: {name}")
        if entry.get("sha256") != _sha256_file(path):
            raise PublicationError(f"artifact SHA-256 mismatch: {name}")

    receipt = _json(source / "full_release_receipt.json", "full release receipt")
    if receipt.get("schema_version") != RECEIPT_SCHEMA:
        raise PublicationError("receipt schema mismatch")
    _require_self_hash(receipt, "receipt_content_sha256", "receipt")
    if receipt.get("classification") != "FORMAL_FULL_RELEASE":
        raise PublicationError("receipt classification is not FORMAL_FULL_RELEASE")
    for key in ("N_eval", "J_eval", "category_count", "sampling_protocol", "protocol_id"):
        if receipt.get(key) != manifest.get(key):
            raise PublicationError(f"receipt/manifest binding mismatch: {key}")
    if receipt.get("sampling_protocol") != SAMPLING_PROTOCOL or receipt.get("protocol_id") != PROTOCOL_ID:
        raise PublicationError("receipt protocol mismatch")
    if progress.get("completed_assets") != receipt.get("N_eval") or progress.get("total_assets") != receipt.get("N_eval"):
        raise PublicationError("progress denominator binding mismatch")
    if checkpoint.get("records") != receipt.get("N_eval"):
        raise PublicationError("checkpoint record denominator mismatch")
    if checkpoint.get("manifest_content_sha256") != manifest.get("manifest_content_sha256"):
        raise PublicationError("checkpoint/manifest binding mismatch")

    bindings = {
        "manifest": ("manifest.json", "manifest_sha256"),
        "records": ("records.jsonl", "records_sha256"),
        "state_records": ("state_records.jsonl", "state_records_sha256"),
        "summary": ("summary.json", "summary_sha256"),
        "artifact_manifest": ("artifact_manifest.json", "artifact_manifest_sha256"),
        "result_database": ("results.sqlite3", "result_database_sha256"),
    }
    for key, (name, hash_key) in bindings.items():
        path = _relative_file(source, receipt.get(key), name, key)
        if receipt.get(hash_key) != _sha256_file(path):
            raise PublicationError(f"receipt file binding mismatch: {key}")
    if checkpoint.get("records_sha256") != receipt.get("records_sha256"):
        raise PublicationError("checkpoint/receipt records binding mismatch")
    if checkpoint.get("state_records_sha256") != receipt.get("state_records_sha256"):
        raise PublicationError("checkpoint/receipt state binding mismatch")
    if checkpoint.get("summary_sha256") != receipt.get("summary_sha256"):
        raise PublicationError("checkpoint/receipt summary binding mismatch")

    for suffix in ("-wal", "-shm", "-journal"):
        sidecar = source / f"results.sqlite3{suffix}"
        try:
            sidecar_mode = sidecar.lstat().st_mode
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(sidecar_mode) or not stat.S_ISREG(sidecar_mode) or sidecar.stat().st_size != 0:
            raise PublicationError(f"non-empty or unsafe SQLite sidecar forbids publication: {sidecar.name}")
    files = {}
    for name in PUBLISHED_FILES:
        path = _regular_file(source / name, f"source {name}")
        files[name] = {"bytes": path.stat().st_size, "sha256": _sha256_file(path)}
    return {
        "manifest": manifest,
        "checkpoint": checkpoint,
        "artifact": artifact,
        "receipt": receipt,
        "progress": progress,
        "files": files,
    }


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_json_new_fsync(path: Path, value: Mapping[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise PublicationError(f"JSON target already exists: {path}")
    data = (_canonical_text(value) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def _renameat2_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(errno.ENOSYS, "renameat2 is unavailable", str(destination))
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        -100,
        os.fsencode(source),
        -100,
        os.fsencode(destination),
        1,
    )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    raise OSError(error_number, os.strerror(error_number), str(destination))


def _rename_via_empty_directory_reservation(source: Path, destination: Path) -> None:
    """Reserve the target name, then atomically replace only that empty directory."""
    parent_descriptor = os.open(
        destination.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    )
    try:
        fcntl.flock(parent_descriptor, fcntl.LOCK_EX)
        source_info = source.lstat()
        try:
            destination.mkdir(mode=0o700)
        except FileExistsError as error:
            raise PublicationError(
                f"destination appeared during publication: {destination}"
            ) from error
        reservation_descriptor = os.open(
            destination,
            os.O_RDONLY
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_NOFOLLOW", 0),
        )
        try:
            reservation_info = os.fstat(reservation_descriptor)
            _fsync_directory(destination.parent)
            current_info = destination.lstat()
            if (
                not stat.S_ISDIR(current_info.st_mode)
                or (current_info.st_dev, current_info.st_ino)
                != (reservation_info.st_dev, reservation_info.st_ino)
                or os.listdir(reservation_descriptor)
            ):
                raise PublicationError("destination reservation changed before rename")
            os.rename(source, destination)
        finally:
            os.close(reservation_descriptor)
        installed_info = destination.lstat()
        if (installed_info.st_dev, installed_info.st_ino) != (
            source_info.st_dev,
            source_info.st_ino,
        ):
            raise PublicationError(
                "reserved-directory rename did not install the source inode"
            )
    finally:
        os.close(parent_descriptor)


def _rename_noreplace(source: Path, destination: Path) -> None:
    """Atomically install a directory without replacing a material target."""
    try:
        _renameat2_noreplace(source, destination)
        return
    except OSError as error:
        if error.errno == errno.EEXIST:
            raise PublicationError(
                f"destination appeared during publication: {destination}"
            ) from error
        unsupported = {
            errno.EINVAL,
            errno.ENOSYS,
            getattr(errno, "ENOTSUP", errno.EOPNOTSUPP),
            errno.EOPNOTSUPP,
        }
        if error.errno not in unsupported:
            raise
    _rename_via_empty_directory_reservation(source, destination)


def _stable_stage_file_binding(path: Path, label: str) -> dict[str, Any]:
    path = _regular_file(path, label)
    before = path.lstat()
    identity_before = (
        before.st_dev,
        before.st_ino,
        before.st_mode,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    )
    digest = _sha256_file(path)
    after = path.lstat()
    identity_after = (
        after.st_dev,
        after.st_ino,
        after.st_mode,
        after.st_size,
        after.st_mtime_ns,
        after.st_ctime_ns,
    )
    if identity_before != identity_after:
        raise PublicationError(f"file changed while hashing: {label}")
    return {
        "bytes": after.st_size,
        "sha256": digest,
        "identity": identity_after,
    }


def _recheck_stage_file_identities(
    stage: Path, observed: Mapping[str, Mapping[str, Any]]
) -> None:
    expected_names = set(PUBLISHED_FILES) | {INTERNAL_COPY_RECEIPT}
    if {path.name for path in stage.iterdir()} != expected_names:
        raise PublicationError("recovery stage file set changed before installation")
    for name in PUBLISHED_FILES:
        info = (stage / name).lstat()
        identity = (
            info.st_dev,
            info.st_ino,
            info.st_mode,
            info.st_size,
            info.st_mtime_ns,
            info.st_ctime_ns,
        )
        if identity != tuple(observed[name]["identity"]):
            raise PublicationError(f"recovery stage changed before installation: {name}")


def _validate_stage_file_rows(
    stage: Path, rows: Any
) -> dict[str, dict[str, Any]]:
    stage = stage.absolute()
    if not stage.is_dir() or stage.is_symlink():
        raise PublicationError(f"recovery stage is not a regular directory: {stage}")
    expected_names = set(PUBLISHED_FILES) | {INTERNAL_COPY_RECEIPT}
    observed_names = {path.name for path in stage.iterdir()}
    if observed_names != expected_names:
        raise PublicationError("recovery stage does not contain the exact published file set")
    if not isinstance(rows, list) or len(rows) != len(PUBLISHED_FILES):
        raise PublicationError("copy receipt must bind exactly twelve files")
    by_name: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != {"path", "bytes", "sha256"}:
            raise PublicationError("invalid copy receipt file row")
        name = row.get("path")
        if not isinstance(name, str) or name in by_name:
            raise PublicationError("duplicate or invalid copy receipt path")
        by_name[name] = row
    if set(by_name) != set(PUBLISHED_FILES):
        raise PublicationError("copy receipt does not bind the exact published file set")
    observed: dict[str, dict[str, Any]] = {}
    for name in PUBLISHED_FILES:
        row = by_name[name]
        binding = _stable_stage_file_binding(
            stage / name, f"recovery stage {name}"
        )
        if {key: binding[key] for key in ("bytes", "sha256")} != {
            "bytes": row.get("bytes"),
            "sha256": row.get("sha256"),
        }:
            raise PublicationError(f"recovery stage binding mismatch: {name}")
        observed[name] = binding
    return observed


def _validate_stage_copy_receipt(
    stage: Path,
    destination: Path,
    *,
    prior_publisher_sha256: str,
) -> dict[str, Any]:
    receipt = _json(stage / INTERNAL_COPY_RECEIPT, "internal copy receipt")
    if receipt.get("schema_version") != COPY_RECEIPT_SCHEMA:
        raise PublicationError("copy receipt schema mismatch")
    _require_self_hash(receipt, "copy_receipt_content_sha256", "copy receipt")
    if Path(str(receipt.get("destination", ""))).absolute() != destination.absolute():
        raise PublicationError("copy receipt destination mismatch")
    if receipt.get("publisher_source") != str(SCRIPT):
        raise PublicationError("copy receipt publisher path mismatch")
    if receipt.get("publisher_source_sha256") != prior_publisher_sha256:
        raise PublicationError("prior publisher hash does not match the failed receipt")
    if receipt.get("checker_source") != str(CHECKER_SOURCE):
        raise PublicationError("copy receipt checker path mismatch")
    if receipt.get("checker_source_sha256") != _sha256_file(CHECKER_SOURCE):
        raise PublicationError("checker source changed since the failed publication")
    if receipt.get("checker_all_pass") is not True:
        raise PublicationError("failed publication did not record a passing checker")
    _validate_stage_file_rows(stage, receipt.get("files"))
    return receipt


def _validate_recovered_copy_receipt(
    root: Path,
    destination: Path,
    backup_path: Path,
    *,
    prior_publisher_sha256: str,
    trigger_errno: int,
    publisher_source_sha256: str,
    checker_source_sha256: str,
) -> dict[str, Any]:
    receipt = _json(root / INTERNAL_COPY_RECEIPT, "recovered internal copy receipt")
    if receipt.get("schema_version") != COPY_RECEIPT_SCHEMA:
        raise PublicationError("recovered copy receipt schema mismatch")
    _require_self_hash(receipt, "copy_receipt_content_sha256", "recovered copy receipt")
    if Path(str(receipt.get("destination", ""))).absolute() != destination.absolute():
        raise PublicationError("recovered copy receipt destination mismatch")
    if (
        receipt.get("publisher_source") != str(SCRIPT)
        or receipt.get("publisher_source_sha256") != publisher_source_sha256
        or receipt.get("checker_source") != str(CHECKER_SOURCE)
        or receipt.get("checker_source_sha256") != checker_source_sha256
        or receipt.get("checker_all_pass") is not True
    ):
        raise PublicationError("recovered copy receipt implementation mismatch")
    recovery = receipt.get("failed_install_recovery")
    if (
        not isinstance(recovery, Mapping)
        or recovery.get("trigger_errno") != trigger_errno
        or recovery.get("prior_publisher_source_sha256")
        != prior_publisher_sha256
    ):
        raise PublicationError("recovered copy receipt recovery metadata mismatch")
    prior_binding = recovery.get("prior_copy_receipt")
    if not isinstance(prior_binding, Mapping):
        raise PublicationError("recovered copy receipt lacks its prior receipt binding")
    backup_binding = _stable_stage_file_binding(
        backup_path, "failed-rename receipt backup"
    )
    if (
        prior_binding.get("path") != str(backup_path)
        or prior_binding.get("bytes") != backup_binding["bytes"]
        or prior_binding.get("sha256") != backup_binding["sha256"]
    ):
        raise PublicationError("failed-rename receipt backup binding mismatch")
    _validate_stage_file_rows(root, receipt.get("files"))
    return receipt


def recover_failed_install(
    destination: Path,
    *,
    prior_publisher_sha256: str,
    trigger_errno: int,
    copy_receipt: Path | None = None,
    receipt_backup: Path | None = None,
) -> Path:
    """Install a fully checked stage after an unsupported renameat2 flag."""
    destination = Path(destination).absolute()
    parent = destination.parent
    stage = parent / f".{destination.name}.publish-stage"
    receipt_path = (
        Path(copy_receipt).absolute()
        if copy_receipt is not None
        else parent / f"{destination.name}.copy_receipt.json"
    )
    backup_path = (
        Path(receipt_backup).absolute()
        if receipt_backup is not None
        else parent / f"{destination.name}.failed_rename_receipt.json"
    )
    if not parent.is_dir() or parent.is_symlink():
        raise PublicationError("destination parent must be an existing non-symlink directory")
    if receipt_path.parent != parent or backup_path.parent != parent:
        raise PublicationError("recovery receipts must be in the destination parent")
    recovery_paths = (destination, stage, receipt_path, backup_path)
    if len({path.resolve(strict=False) for path in recovery_paths}) != len(
        recovery_paths
    ):
        raise PublicationError("recovery paths must be pairwise distinct")
    if trigger_errno not in {
        errno.EINVAL,
        errno.ENOSYS,
        getattr(errno, "ENOTSUP", errno.EOPNOTSUPP),
        errno.EOPNOTSUPP,
    }:
        raise PublicationError("recovery trigger is not an unsupported renameat2 error")

    publisher_source_hash = _sha256_file(SCRIPT)
    checker_source_hash = _sha256_file(CHECKER_SOURCE)
    reservation_present = False
    if destination.exists() or destination.is_symlink():
        if destination.is_symlink() or not destination.is_dir():
            raise PublicationError(f"destination already exists: {destination}")
        if stage.exists() or stage.is_symlink():
            if (
                stage.is_symlink()
                or any(destination.iterdir())
                or receipt_path.exists()
                or receipt_path.is_symlink()
                or not backup_path.is_file()
                or backup_path.is_symlink()
            ):
                raise PublicationError("ambiguous partial recovery state")
            reservation_present = True
        else:
            if not backup_path.is_file() or backup_path.is_symlink():
                raise PublicationError("installed recovery lacks its prior receipt backup")
            recovered = _validate_recovered_copy_receipt(
                destination,
                destination,
                backup_path,
                prior_publisher_sha256=prior_publisher_sha256,
                trigger_errno=trigger_errno,
                publisher_source_sha256=publisher_source_hash,
                checker_source_sha256=checker_source_hash,
            )
            installed_receipt = destination / INTERNAL_COPY_RECEIPT
            if receipt_path.exists() or receipt_path.is_symlink():
                if (
                    receipt_path.is_symlink()
                    or not receipt_path.is_file()
                    or not os.path.samefile(installed_receipt, receipt_path)
                ):
                    raise PublicationError("external copy receipt differs after recovery")
            else:
                os.link(installed_receipt, receipt_path)
                _fsync_directory(parent)
            if (
                _sha256_file(SCRIPT) != publisher_source_hash
                or _sha256_file(CHECKER_SOURCE) != checker_source_hash
            ):
                raise PublicationError("publisher or checker source changed during recovery")
            return receipt_path
    if receipt_path.exists() or receipt_path.is_symlink():
        raise PublicationError(f"copy receipt already exists: {receipt_path}")

    internal_receipt = stage / INTERNAL_COPY_RECEIPT
    current_receipt = _json(internal_receipt, "internal copy receipt")
    if current_receipt.get("publisher_source_sha256") == prior_publisher_sha256:
        old_receipt = _validate_stage_copy_receipt(
            stage,
            destination,
            prior_publisher_sha256=prior_publisher_sha256,
        )
        if backup_path.exists() or backup_path.is_symlink():
            backup = _regular_file(backup_path, "failed-rename receipt backup")
            if backup.read_bytes() != internal_receipt.read_bytes():
                raise PublicationError("existing failed-rename receipt backup differs")
        else:
            os.link(internal_receipt, backup_path)
            _fsync_directory(parent)
        backup_binding = {
            "path": str(backup_path),
            "bytes": backup_path.stat().st_size,
            "sha256": _sha256_file(backup_path),
        }
        value = dict(old_receipt)
        value["publisher_source"] = str(SCRIPT)
        value["publisher_source_sha256"] = publisher_source_hash
        value["checker_source"] = str(CHECKER_SOURCE)
        value["checker_source_sha256"] = checker_source_hash
        value["install_strategy"] = (
            "renameat2_noreplace_with_reserved_empty_directory_fallback_v1"
        )
        value["failed_install_recovery"] = {
            "trigger_operation": "renameat2(RENAME_NOREPLACE)",
            "trigger_errno": trigger_errno,
            "trigger_error": os.strerror(trigger_errno),
            "prior_publisher_source_sha256": prior_publisher_sha256,
            "prior_copy_receipt": backup_binding,
        }
        value["copy_receipt_content_sha256"] = _self_hash(
            value, "copy_receipt_content_sha256"
        )
        temporary_receipt = stage / ".publication_copy_receipt.recovery.tmp"
        _write_json_new_fsync(temporary_receipt, value)
        os.replace(temporary_receipt, internal_receipt)
        _fsync_directory(stage)
    elif current_receipt.get("publisher_source_sha256") == publisher_source_hash:
        value = _validate_recovered_copy_receipt(
            stage,
            destination,
            backup_path,
            prior_publisher_sha256=prior_publisher_sha256,
            trigger_errno=trigger_errno,
            publisher_source_sha256=publisher_source_hash,
            checker_source_sha256=checker_source_hash,
        )
    else:
        raise PublicationError("stage receipt has an unknown publisher binding")

    if reservation_present:
        os.rmdir(destination)
        _fsync_directory(parent)

    # Close the long hashing window before the namespace installation.
    final_stage_bindings = _validate_stage_file_rows(stage, value["files"])
    recovered_receipt = _json(internal_receipt, "recovered internal copy receipt")
    _require_self_hash(
        recovered_receipt, "copy_receipt_content_sha256", "recovered copy receipt"
    )
    if recovered_receipt != value:
        raise PublicationError("recovered copy receipt changed before installation")
    if (
        _sha256_file(SCRIPT) != publisher_source_hash
        or _sha256_file(CHECKER_SOURCE) != checker_source_hash
    ):
        raise PublicationError("publisher or checker source changed during recovery")
    _recheck_stage_file_identities(stage, final_stage_bindings)
    _rename_noreplace(stage, destination)
    _fsync_directory(parent)
    installed_receipt = destination / INTERNAL_COPY_RECEIPT
    try:
        os.link(installed_receipt, receipt_path)
        _fsync_directory(parent)
    except FileExistsError as error:
        raise PublicationError(
            f"copy receipt appeared during recovery: {receipt_path}"
        ) from error
    return receipt_path


def publish(source: Path, destination: Path, *, copy_receipt: Path | None = None) -> Path:
    source = Path(source).absolute()
    destination = Path(destination).absolute()
    if source == destination:
        raise PublicationError("source and destination must differ")
    parent = destination.parent
    if not parent.is_dir() or parent.is_symlink():
        raise PublicationError("destination parent must be an existing non-symlink directory")
    stage = parent / f".{destination.name}.publish-stage"
    receipt_path = Path(copy_receipt).absolute() if copy_receipt is not None else parent / f"{destination.name}.copy_receipt.json"
    if receipt_path.parent != parent:
        raise PublicationError("copy receipt must be in the destination parent directory")
    if receipt_path in {destination, stage}:
        raise PublicationError("copy receipt must not alias destination or stage")
    resolved_source = source.resolve(strict=True)
    resolved_parent = parent.resolve(strict=True)
    if resolved_parent == resolved_source or resolved_source in resolved_parent.parents:
        raise PublicationError("destination and stage must not be inside the read-only source")
    for path, label in ((destination, "destination"), (stage, "stage"), (receipt_path, "copy receipt")):
        if path.exists() or path.is_symlink():
            raise PublicationError(f"{label} already exists: {path}")

    publisher_source_hash = _sha256_file(SCRIPT)
    checker_source_hash = _sha256_file(CHECKER_SOURCE)
    verified = verify_source(source)
    checker_report = checker.check(source)
    if checker_report.get("all_pass") is not True:
        raise PublicationError("independent full-release checker did not pass")
    expected_files = verified["files"]
    stage.mkdir(mode=0o755)
    copied: list[dict[str, Any]] = []
    try:
        for name in PUBLISHED_FILES:
            src = _regular_file(source / name, f"source {name}")
            dst = stage / name
            with src.open("rb") as reader, dst.open("xb") as writer:
                shutil.copyfileobj(reader, writer, length=4 * 1024 * 1024)
                writer.flush()
                os.fsync(writer.fileno())
            source_size, destination_size = src.stat().st_size, dst.stat().st_size
            source_hash, destination_hash = _sha256_file(src), _sha256_file(dst)
            expected = expected_files[name]
            if (
                source_size != expected["bytes"]
                or source_hash != expected["sha256"]
                or destination_size != expected["bytes"]
                or destination_hash != expected["sha256"]
            ):
                raise PublicationError(f"post-copy verification failed: {name}")
            copied.append({"path": name, "bytes": source_size, "sha256": source_hash})
        if verify_source(source)["files"] != expected_files:
            raise PublicationError("source changed while the sealed snapshot was copied")
        stage_checker_report = checker.check(stage)
        if stage_checker_report.get("all_pass") is not True:
            raise PublicationError("independent checker rejected the copied stage")
        if (
            _sha256_file(SCRIPT) != publisher_source_hash
            or _sha256_file(CHECKER_SOURCE) != checker_source_hash
        ):
            raise PublicationError("publisher or checker source changed during publication")
        value: dict[str, Any] = {
            "schema_version": COPY_RECEIPT_SCHEMA,
            "source": str(source),
            "destination": str(destination),
            "files": copied,
            "publisher_source": str(SCRIPT),
            "publisher_source_sha256": publisher_source_hash,
            "checker_source": str(CHECKER_SOURCE),
            "checker_source_sha256": checker_source_hash,
            "checker_all_pass": True,
            "install_strategy": (
                "renameat2_noreplace_with_reserved_empty_directory_fallback_v1"
            ),
        }
        value["copy_receipt_content_sha256"] = _self_hash(value, "copy_receipt_content_sha256")
        _write_json_new_fsync(stage / INTERNAL_COPY_RECEIPT, value)
        _fsync_directory(stage)
        final_stage_checker_report = checker.check(stage)
        if final_stage_checker_report.get("all_pass") is not True:
            raise PublicationError("independent checker rejected stage with publication receipt")
        if (
            _sha256_file(SCRIPT) != publisher_source_hash
            or _sha256_file(CHECKER_SOURCE) != checker_source_hash
        ):
            raise PublicationError("publisher or checker source changed during publication")
        _rename_noreplace(stage, destination)
        _fsync_directory(parent)
    except BaseException:
        # Deliberately preserve a failed stage for inspection; never delete it.
        raise

    internal_receipt = destination / INTERNAL_COPY_RECEIPT
    try:
        # link(2) is an atomic no-clobber install and guarantees byte identity.
        os.link(internal_receipt, receipt_path)
        _fsync_directory(parent)
    except FileExistsError as error:
        raise PublicationError(f"copy receipt appeared during publication: {receipt_path}") from error
    return receipt_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--copy-receipt", type=Path)
    parser.add_argument(
        "--resume-failed-install",
        action="store_true",
        help="recover a fully checked stage after unsupported renameat2 flags",
    )
    parser.add_argument("--prior-publisher-sha256")
    parser.add_argument("--trigger-errno", type=int)
    parser.add_argument("--failed-receipt-backup", type=Path)
    args = parser.parse_args(argv)
    if args.resume_failed_install:
        if args.source is not None:
            parser.error("--source cannot be used with --resume-failed-install")
        if args.prior_publisher_sha256 is None or args.trigger_errno is None:
            parser.error(
                "--resume-failed-install requires --prior-publisher-sha256 and --trigger-errno"
            )
        receipt = recover_failed_install(
            args.destination,
            prior_publisher_sha256=args.prior_publisher_sha256,
            trigger_errno=args.trigger_errno,
            copy_receipt=args.copy_receipt,
            receipt_backup=args.failed_receipt_backup,
        )
    else:
        if args.source is None:
            parser.error("--source is required unless --resume-failed-install is used")
        if (
            args.prior_publisher_sha256 is not None
            or args.trigger_errno is not None
            or args.failed_receipt_backup is not None
        ):
            parser.error("recovery-only arguments require --resume-failed-install")
        receipt = publish(args.source, args.destination, copy_receipt=args.copy_receipt)
    print(receipt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
