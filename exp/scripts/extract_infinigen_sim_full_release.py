#!/usr/bin/env python3
"""Securely stage the Infinigen-Sim URDF tar archives."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
import tarfile
from pathlib import Path, PurePosixPath
from typing import Any


# Frozen archive bindings from the local HF release inventory.  Callers may
# pass a different map for a separately pinned release, but production runs
# must provide all archive hashes before extraction.
EXPECTED_ARCHIVE_SHA256: dict[str, str] = {}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _member_path(destination: Path, name: str) -> Path:
    pure = PurePosixPath(name)
    if pure.is_absolute() or not pure.parts or any(part in {"", ".", ".."} for part in pure.parts):
        raise ValueError(f"archive path traversal: {name}")
    target = (destination / Path(*pure.parts)).resolve()
    try:
        target.relative_to(destination.resolve())
    except ValueError as exc:
        raise ValueError(f"archive path traversal: {name}") from exc
    return target


def _validate_member(member: tarfile.TarInfo, destination: Path) -> Path:
    target = _member_path(destination, member.name)
    if member.issym() or member.islnk():
        raise ValueError(f"archive link member rejected: {member.name}")
    if member.ischr() or member.isblk() or member.isfifo() or not (member.isdir() or member.isfile()):
        raise ValueError(f"archive special member rejected: {member.name}")
    return target


def _extract_one(archive: Path, destination: Path) -> dict[str, Any]:
    archive_name = archive.name[:-7] if archive.name.endswith(".tar.gz") else archive.stem
    staging = destination / archive_name
    if staging.exists():
        raise ValueError(f"refusing to merge into existing staging directory: {staging}")
    staging.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        targets = [_validate_member(member, staging) for member in members]
        target_names = [target.relative_to(staging).as_posix() for target in targets]
        if len(target_names) != len(set(target_names)):
            raise ValueError(f"archive contains duplicate member paths: {archive.name}")
        for member, target in zip(members, targets):
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            source = tar.extractfile(member)
            if source is None:
                raise ValueError(f"archive member has no payload: {member.name}")
            with target.open("wb") as stream:
                shutil.copyfileobj(source, stream)
            os.chmod(target, member.mode & 0o777)
    return {"name": archive.name, "path": str(archive.resolve()), "size": archive.stat().st_size, "sha256": sha256_file(archive), "member_count": len(members)}


def _validate_one_read_only(
    archive: Path,
    *,
    expected_sha256: str | None = None,
) -> dict[str, Any]:
    """Validate an archive's member graph without extracting or writing it."""

    actual_sha256 = sha256_file(archive)
    if expected_sha256 is not None and actual_sha256 != expected_sha256:
        raise ValueError(f"archive hash mismatch: {archive.name}")
    # A non-existent destination is sufficient for path validation; the
    # validator never creates it or touches archive payloads on disk.
    destination = archive.parent / ".read_only_validation_target"
    seen: set[str] = set()
    counts = {"files": 0, "directories": 0, "links": 0, "special": 0}
    total_bytes = 0
    with tarfile.open(archive, "r:gz") as tar:
        for member in tar:
            _validate_member(member, destination)
            normalized = PurePosixPath(member.name).as_posix()
            if normalized in seen:
                raise ValueError(f"archive contains duplicate member paths: {archive.name}")
            seen.add(normalized)
            if member.isdir():
                counts["directories"] += 1
            elif member.isfile():
                counts["files"] += 1
                total_bytes += int(member.size)
            elif member.issym() or member.islnk():
                counts["links"] += 1
            else:
                counts["special"] += 1
    return {
        "name": archive.name,
        "path": str(archive.resolve()),
        "size": archive.stat().st_size,
        "sha256": actual_sha256,
        "member_count": len(seen),
        "member_counts": counts,
        "member_payload_bytes": total_bytes,
    }


def validate_archives_read_only(
    archive_root: Path,
    *,
    expected_hashes: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Return a deterministic, source-bound validation receipt for archives."""

    root = Path(archive_root)
    if not root.is_dir():
        raise ValueError(f"archive root is not a directory: {root}")
    archives = sorted(root.glob("*.tar.gz"))
    expected = EXPECTED_ARCHIVE_SHA256 if expected_hashes is None else expected_hashes
    if expected and set(expected) != {path.name for path in archives}:
        missing = sorted(set(expected) - {path.name for path in archives})
        extra = sorted({path.name for path in archives} - set(expected))
        detail = missing[0] if missing else extra[0]
        raise ValueError(f"archive inventory mismatch: {detail}")
    records = [
        _validate_one_read_only(path, expected_sha256=(expected or {}).get(path.name))
        for path in archives
    ]
    receipt: dict[str, Any] = {
        "schema_version": "infinigen_archive_validation_v1",
        "archive_root": str(root.resolve()),
        "archive_count": len(records),
        "archives": records,
    }
    receipt["receipt_content_sha256"] = hashlib.sha256(
        _canonical_receipt_bytes(receipt)
    ).hexdigest()
    return receipt


def _canonical_receipt_bytes(value: dict[str, Any]) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def extract_archives_securely(
    archive_root: Path,
    destination: Path,
    *,
    expected_hashes: dict[str, str] | None = None,
) -> dict[str, Any]:
    root = Path(archive_root); destination = Path(destination)
    if not root.is_dir():
        raise ValueError(f"archive root is not a directory: {root}")
    if destination.exists() and destination.is_symlink():
        raise ValueError(f"destination must not be a symlink: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    archives = sorted(root.glob("*.tar.gz"))
    expected = EXPECTED_ARCHIVE_SHA256 if expected_hashes is None else expected_hashes
    records = []
    for path in archives:
        if expected:
            declared = expected.get(path.name)
            if declared is None:
                raise ValueError(f"archive hash is not pinned: {path.name}")
            actual = sha256_file(path)
            if actual != declared:
                raise ValueError(f"archive hash mismatch: {path.name}")
        records.append(_extract_one(path, destination))
    if expected and set(expected) != {path.name for path in archives}:
        missing = sorted(set(expected) - {path.name for path in archives})
        raise ValueError(f"pinned archive is missing: {missing[0]}")
    return {"schema_version": "infinigen_secure_extraction_v1", "archive_count": len(records), "archives": records, "archives_sha256": hashlib.sha256("".join(record["sha256"] for record in records).encode()).hexdigest()}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("archive_root", type=Path)
    parser.add_argument("destination", type=Path, nargs="?")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--expected-sha256-file", type=Path)
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args(argv)
    expected: dict[str, str] = {}
    if args.expected_sha256_file:
        for line in args.expected_sha256_file.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            digest, name = line.split(None, 1)
            expected[Path(name).name] = digest
    if args.validate_only:
        receipt = validate_archives_read_only(args.archive_root, expected_hashes=expected or None)
        payload = json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=True) + "\n"
        if args.receipt:
            args.receipt.parent.mkdir(parents=True, exist_ok=True)
            args.receipt.write_text(payload, encoding="utf-8")
        else:
            print(payload, end="")
        return 0
    if args.destination is None:
        parser.error("destination is required unless --validate-only is used")
    extract_archives_securely(args.archive_root, args.destination, expected_hashes=expected or None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
