#!/usr/bin/env python3
"""Pack and reconstruct SketchMobility using verified ``tar.gz`` chunks.

The repository layout follows the Artiverse release convention:

    dataset_chunks/
      manifest.json
      sketchmobility_data-00001-of-00003.tar.gz
      ...

Archives contain a top-level ``data/`` directory. Object directories are kept
whole, so no object's URDF, annotation, or mesh bundle is split across chunks.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import shutil
import tarfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO, Iterable


MANIFEST_NAME = "manifest.json"
KNOWN_SOURCES = ("Agentic", "Infinigen", "PartNeXt", "Shape2Motion")


@dataclass(frozen=True)
class ObjectBundle:
    relative: Path
    files: tuple[Path, ...]
    directories: tuple[Path, ...]
    size_bytes: int


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _discover_objects(data_root: Path) -> list[ObjectBundle]:
    bundles: list[ObjectBundle] = []
    for source in KNOWN_SOURCES:
        source_root = data_root / source
        if not source_root.is_dir():
            continue
        for category_root in sorted(path for path in source_root.iterdir() if path.is_dir()):
            for object_root in sorted(path for path in category_root.iterdir() if path.is_dir()):
                relative = object_root.relative_to(data_root)
                required = (object_root / "annotation.json", object_root / "mobility.urdf")
                if not all(path.is_file() for path in required):
                    raise ValueError(f"Incomplete object bundle: {relative}")
                symlinks = sorted(path for path in object_root.rglob("*") if path.is_symlink())
                if symlinks:
                    raise ValueError(f"Symlinks are not supported in release bundles: {symlinks[0]}")
                files = tuple(sorted(path for path in object_root.rglob("*") if path.is_file()))
                directories = tuple(sorted(path for path in object_root.rglob("*") if path.is_dir()))
                bundles.append(
                    ObjectBundle(
                        relative=relative,
                        files=files,
                        directories=directories,
                        size_bytes=sum(path.stat().st_size for path in files),
                    )
                )
    if not bundles:
        raise ValueError(f"No release objects found under {data_root}")
    return bundles


def _partition(
    bundles: Iterable[ObjectBundle],
    target_bytes: int,
) -> list[list[ObjectBundle]]:
    if target_bytes <= 0:
        raise ValueError("Chunk target size must be positive")
    chunks: list[list[ObjectBundle]] = []
    current: list[ObjectBundle] = []
    current_bytes = 0
    for bundle in bundles:
        if current and current_bytes + bundle.size_bytes > target_bytes:
            chunks.append(current)
            current = []
            current_bytes = 0
        current.append(bundle)
        current_bytes += bundle.size_bytes
    if current:
        chunks.append(current)
    return chunks


def _normalized_tarinfo(name: str, *, size: int = 0, directory: bool = False) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name=name)
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = 0
    info.mode = 0o755 if directory else 0o644
    if directory:
        info.type = tarfile.DIRTYPE
    else:
        info.type = tarfile.REGTYPE
        info.size = size
    return info


def _archive_name(prefix: str, index: int, total: int) -> str:
    return f"{prefix}-{index:05d}-of-{total:05d}.tar.gz"


def _write_archive(
    target: Path,
    bundles: list[ObjectBundle],
    *,
    data_root: Path,
    root_name: str,
    compresslevel: int,
) -> None:
    partial = target.with_name(f".{target.name}.partial")
    if partial.exists():
        partial.unlink()
    with partial.open("wb") as raw:
        with gzip.GzipFile(
            filename="",
            mode="wb",
            compresslevel=compresslevel,
            fileobj=raw,
            mtime=0,
        ) as compressed:
            with tarfile.open(fileobj=compressed, mode="w|", format=tarfile.PAX_FORMAT) as archive:
                for bundle in bundles:
                    object_member = PurePosixPath(root_name) / PurePosixPath(bundle.relative.as_posix())
                    archive.addfile(
                        _normalized_tarinfo(object_member.as_posix(), directory=True)
                    )
                    for directory in bundle.directories:
                        relative = directory.relative_to(data_root)
                        member = PurePosixPath(root_name) / PurePosixPath(relative.as_posix())
                        archive.addfile(_normalized_tarinfo(member.as_posix(), directory=True))
                    for source in bundle.files:
                        relative = source.relative_to(data_root)
                        member = PurePosixPath(root_name) / PurePosixPath(relative.as_posix())
                        with source.open("rb") as payload:
                            archive.addfile(
                                _normalized_tarinfo(member.as_posix(), size=source.stat().st_size),
                                payload,
                            )
        raw.flush()
        os.fsync(raw.fileno())
    os.replace(partial, target)


def pack(
    data_root: Path,
    chunks_dir: Path,
    *,
    chunk_size_bytes: int,
    prefix: str,
    root_name: str,
    compresslevel: int,
    overwrite: bool,
) -> dict[str, Any]:
    data_root = data_root.resolve()
    chunks_dir = chunks_dir.resolve()
    if not data_root.is_dir():
        raise FileNotFoundError(f"Dataset root does not exist: {data_root}")
    if not root_name or "/" in root_name or root_name in {".", ".."}:
        raise ValueError(f"Unsafe archive root name: {root_name!r}")
    bundles = _discover_objects(data_root)
    chunk_groups = _partition(bundles, chunk_size_bytes)
    chunks_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(chunks_dir.glob(f"{prefix}-*-of-*.tar.gz"))
    manifest_path = chunks_dir / MANIFEST_NAME
    if (existing or manifest_path.exists()) and not overwrite:
        raise FileExistsError(f"Chunk output already exists under {chunks_dir}")
    if overwrite:
        for path in existing:
            path.unlink()
        if manifest_path.exists():
            manifest_path.unlink()

    chunk_records: list[dict[str, Any]] = []
    object_records: list[dict[str, Any]] = []
    total_chunks = len(chunk_groups)
    for index, group in enumerate(chunk_groups, start=1):
        filename = _archive_name(prefix, index, total_chunks)
        target = chunks_dir / filename
        _write_archive(
            target,
            group,
            data_root=data_root,
            root_name=root_name,
            compresslevel=compresslevel,
        )
        for bundle in group:
            object_records.append(
                {
                    "path": (PurePosixPath(root_name) / bundle.relative.as_posix()).as_posix(),
                    "chunk": filename,
                    "fileCount": len(bundle.files),
                    "uncompressedBytes": bundle.size_bytes,
                }
            )
        chunk_records.append(
            {
                "filename": filename,
                "sha256": _sha256(target),
                "sizeBytes": target.stat().st_size,
                "objectCount": len(group),
                "fileCount": sum(len(bundle.files) for bundle in group),
                "uncompressedBytes": sum(bundle.size_bytes for bundle in group),
                "firstObject": group[0].relative.as_posix(),
                "lastObject": group[-1].relative.as_posix(),
            }
        )
        print(
            f"packed {index}/{total_chunks}: {filename} "
            f"({len(group)} objects, {target.stat().st_size / 1024**3:.2f} GiB)"
        )

    manifest: dict[str, Any] = {
        "schemaVersion": 1,
        "dataset": "SketchMobility",
        "archiveFormat": "tar.gz",
        "archiveRoot": root_name,
        "createdAt": datetime.now(timezone.utc).isoformat(),
        "objectCount": len(bundles),
        "fileCount": sum(len(bundle.files) for bundle in bundles),
        "uncompressedBytes": sum(bundle.size_bytes for bundle in bundles),
        "targetChunkBytes": chunk_size_bytes,
        "chunks": chunk_records,
        "objects": object_records,
    }
    temporary = manifest_path.with_name(f".{MANIFEST_NAME}.partial")
    temporary.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, manifest_path)
    return manifest


def _load_manifest(chunks_dir: Path) -> dict[str, Any]:
    path = chunks_dir / MANIFEST_NAME
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schemaVersion") != 1:
        raise ValueError(f"Unsupported manifest: {path}")
    chunks = payload.get("chunks")
    if not isinstance(chunks, list) or not chunks:
        raise ValueError(f"Manifest contains no chunks: {path}")
    return payload


def verify(chunks_dir: Path, *, verify_sha256: bool) -> dict[str, Any]:
    chunks_dir = chunks_dir.resolve()
    manifest = _load_manifest(chunks_dir)
    verified = 0
    for record in manifest["chunks"]:
        if not isinstance(record, dict):
            raise ValueError("Invalid chunk record")
        filename = str(record.get("filename") or "")
        if Path(filename).name != filename:
            raise ValueError(f"Unsafe chunk filename: {filename!r}")
        path = chunks_dir / filename
        if not path.is_file():
            raise FileNotFoundError(f"Missing chunk: {path}")
        expected_size = int(record.get("sizeBytes") or -1)
        if path.stat().st_size != expected_size:
            raise ValueError(f"Chunk size mismatch: {filename}")
        if verify_sha256 and _sha256(path) != str(record.get("sha256") or ""):
            raise ValueError(f"Chunk SHA-256 mismatch: {filename}")
        verified += 1
    return {"chunkCount": verified, "sha256Verified": verify_sha256}


def _safe_member_path(output_dir: Path, name: str) -> Path:
    pure = PurePosixPath(name)
    if pure.is_absolute() or not pure.parts or ".." in pure.parts:
        raise ValueError(f"Unsafe archive member: {name!r}")
    target = (output_dir / Path(*pure.parts)).resolve()
    target.relative_to(output_dir.resolve())
    return target


def _extract_archive(archive_path: Path, output_dir: Path, *, overwrite: bool) -> None:
    with tarfile.open(archive_path, mode="r:gz") as archive:
        for member in archive:
            target = _safe_member_path(output_dir, member.name)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            if not member.isfile():
                raise ValueError(f"Unsupported archive member type: {member.name}")
            if target.exists() and not overwrite:
                raise FileExistsError(f"Refusing to overwrite: {target}")
            target.parent.mkdir(parents=True, exist_ok=True)
            payload = archive.extractfile(member)
            if payload is None:
                raise ValueError(f"Cannot read archive member: {member.name}")
            temporary = target.with_name(f".{target.name}.partial")
            with temporary.open("wb") as handle:
                shutil.copyfileobj(payload, handle, length=8 * 1024 * 1024)
            os.replace(temporary, target)
            target.chmod(member.mode & 0o777)


def unpack(
    chunks_dir: Path,
    output_dir: Path,
    *,
    overwrite: bool,
    verify_sha256: bool,
) -> dict[str, Any]:
    chunks_dir = chunks_dir.resolve()
    output_dir = output_dir.resolve()
    manifest = _load_manifest(chunks_dir)
    verification = verify(chunks_dir, verify_sha256=verify_sha256)
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, record in enumerate(manifest["chunks"], start=1):
        filename = str(record["filename"])
        _extract_archive(chunks_dir / filename, output_dir, overwrite=overwrite)
        print(f"unpacked {index}/{len(manifest['chunks'])}: {filename}")
    return {
        **verification,
        "outputDir": str(output_dir),
        "objectCount": int(manifest.get("objectCount") or 0),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    pack_parser = commands.add_parser("pack", help="Create verified tar.gz chunks")
    pack_parser.add_argument("--data-root", type=Path, required=True)
    pack_parser.add_argument("--chunks-dir", type=Path, required=True)
    pack_parser.add_argument("--chunk-size-gib", type=float, default=5.0)
    pack_parser.add_argument("--prefix", default="sketchmobility_data")
    pack_parser.add_argument("--root-name", default="data")
    pack_parser.add_argument("--compresslevel", type=int, choices=range(1, 10), default=6)
    pack_parser.add_argument("--overwrite", action="store_true")

    verify_parser = commands.add_parser("verify", help="Validate chunk files against the manifest")
    verify_parser.add_argument("--chunks-dir", type=Path, required=True)
    verify_parser.add_argument("--verify-sha256", action="store_true")

    unpack_parser = commands.add_parser("unpack", help="Reconstruct data/ from chunks")
    unpack_parser.add_argument("--chunks-dir", type=Path, required=True)
    unpack_parser.add_argument("--output-dir", type=Path, default=Path("."))
    unpack_parser.add_argument("--overwrite", action="store_true")
    unpack_parser.add_argument("--verify-sha256", action="store_true")
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.command == "pack":
        manifest = pack(
            args.data_root,
            args.chunks_dir,
            chunk_size_bytes=int(args.chunk_size_gib * 1024**3),
            prefix=args.prefix,
            root_name=args.root_name,
            compresslevel=args.compresslevel,
            overwrite=args.overwrite,
        )
        result = {
            "objectCount": manifest["objectCount"],
            "fileCount": manifest["fileCount"],
            "chunkCount": len(manifest["chunks"]),
            "compressedBytes": sum(chunk["sizeBytes"] for chunk in manifest["chunks"]),
        }
    elif args.command == "verify":
        result = verify(args.chunks_dir, verify_sha256=args.verify_sha256)
    else:
        result = unpack(
            args.chunks_dir,
            args.output_dir,
            overwrite=args.overwrite,
            verify_sha256=args.verify_sha256,
        )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
