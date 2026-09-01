#!/usr/bin/env python3
"""Pack and unpack the Artiverse data/ directory as tar.gz chunks.

The packer groups complete model directories into archives whose estimated
uncompressed tar payload is at most --max-chunk-gb. This keeps each chunk
self-contained and makes unpacking order-independent.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tarfile
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_MAX_CHUNK_GB = 45.0
DEFAULT_OUTPUT_DIR = Path("dataset_chunks")
DEFAULT_PREFIX = "artiverse_data"
MANIFEST_NAME = "manifest.json"


@dataclass
class ModelDir:
    rel_path: Path
    size_bytes: int
    file_count: int
    estimated_tar_bytes: int


def iter_files(root: Path):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames.sort()
        filenames.sort()
        for filename in filenames:
            yield Path(dirpath) / filename


def file_sha256(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def model_dirs(data_dir: Path) -> list[Path]:
    models: list[Path] = []
    if not data_dir.is_dir():
        raise FileNotFoundError(f"data directory not found: {data_dir}")
    for category in sorted(p for p in data_dir.iterdir() if p.is_dir()):
        for source in sorted(p for p in category.iterdir() if p.is_dir()):
            for model in sorted(p for p in source.iterdir() if p.is_dir()):
                models.append(model)
    return models


def measure_model(model_path: Path, repo_root: Path) -> ModelDir:
    size_bytes = 0
    file_count = 0
    for file_path in iter_files(model_path):
        if file_path.is_symlink():
            continue
        stat = file_path.stat()
        size_bytes += stat.st_size
        file_count += 1

    # Tar stores 512-byte headers and pads file payloads to 512-byte records.
    # Add a small gzip/header margin so incompressible files do not exceed the
    # nominal chunk cap just from archive overhead.
    estimated_tar_bytes = 0
    for file_path in iter_files(model_path):
        if file_path.is_symlink():
            continue
        stat = file_path.stat()
        estimated_tar_bytes += 512 + ((stat.st_size + 511) // 512) * 512
    estimated_tar_bytes += 1024 * max(file_count, 1)

    return ModelDir(
        rel_path=model_path.relative_to(repo_root),
        size_bytes=size_bytes,
        file_count=file_count,
        estimated_tar_bytes=estimated_tar_bytes,
    )


def build_chunks(models: list[ModelDir], max_bytes: int) -> list[list[ModelDir]]:
    chunks: list[list[ModelDir]] = []
    current: list[ModelDir] = []
    current_bytes = 0
    for model in models:
        if model.estimated_tar_bytes > max_bytes:
            raise ValueError(
                f"single model exceeds chunk limit: {model.rel_path} "
                f"estimated={model.estimated_tar_bytes} limit={max_bytes}"
            )
        if current and current_bytes + model.estimated_tar_bytes > max_bytes:
            chunks.append(current)
            current = []
            current_bytes = 0
        current.append(model)
        current_bytes += model.estimated_tar_bytes
    if current:
        chunks.append(current)
    return chunks


def add_model_to_tar(tar: tarfile.TarFile, repo_root: Path, model: ModelDir) -> None:
    model_abs = repo_root / model.rel_path
    tar.add(model_abs, arcname=model.rel_path.as_posix(), recursive=True)


def write_archive(
    archive_path: Path,
    repo_root: Path,
    models: list[ModelDir],
    compresslevel: int,
    dry_run: bool,
) -> dict:
    print(f"[PACK] {archive_path} ({len(models)} model dirs)")
    if dry_run:
        return {
            "archive": archive_path.name,
            "sha256": None,
            "archive_bytes": None,
            "model_count": len(models),
            "file_count": sum(m.file_count for m in models),
            "input_bytes": sum(m.size_bytes for m in models),
            "estimated_tar_bytes": sum(m.estimated_tar_bytes for m in models),
            "roots": [m.rel_path.as_posix() for m in models],
        }

    archive_path.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive_path, mode="w:gz", compresslevel=compresslevel) as tar:
        for model in models:
            add_model_to_tar(tar, repo_root, model)

    return {
        "archive": archive_path.name,
        "sha256": file_sha256(archive_path),
        "archive_bytes": archive_path.stat().st_size,
        "model_count": len(models),
        "file_count": sum(m.file_count for m in models),
        "input_bytes": sum(m.size_bytes for m in models),
        "estimated_tar_bytes": sum(m.estimated_tar_bytes for m in models),
        "roots": [m.rel_path.as_posix() for m in models],
    }


def pack(args: argparse.Namespace) -> int:
    repo_root = args.repo_root.resolve()
    data_dir = (repo_root / args.data_dir).resolve()
    output_dir = (repo_root / args.output_dir).resolve()
    max_bytes = int(args.max_chunk_gb * 1024**3)

    if not str(data_dir).startswith(str(repo_root)):
        raise ValueError(f"data dir must be inside repo root: {data_dir}")

    models = [measure_model(path, repo_root) for path in model_dirs(data_dir)]
    chunks = build_chunks(models, max_bytes)

    print(f"Repo root: {repo_root}")
    print(f"Data dir: {data_dir}")
    print(f"Output dir: {output_dir}")
    print(f"Model dirs: {len(models)}")
    print(f"Chunks: {len(chunks)}")
    print(f"Max chunk GB: {args.max_chunk_gb}")

    if output_dir.exists() and any(output_dir.iterdir()) and not args.overwrite:
        print(f"error: output dir is not empty; use --overwrite: {output_dir}", file=sys.stderr)
        return 1
    if output_dir.exists() and args.overwrite and not args.dry_run:
        shutil.rmtree(output_dir)
    if not args.dry_run:
        output_dir.mkdir(parents=True, exist_ok=True)

    entries = []
    width = max(5, len(str(len(chunks))))
    for index, chunk_models in enumerate(chunks, start=1):
        archive_name = f"{args.prefix}-{index:0{width}d}-of-{len(chunks):0{width}d}.tar.gz"
        entry = write_archive(
            output_dir / archive_name,
            repo_root,
            chunk_models,
            args.compresslevel,
            args.dry_run,
        )
        entry["index"] = index
        entries.append(entry)

    manifest = {
        "format": "artiverse-data-tar-gz-chunks-v1",
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "data_dir": args.data_dir.as_posix(),
        "max_chunk_gb": args.max_chunk_gb,
        "chunk_count": len(entries),
        "model_count": len(models),
        "file_count": sum(m.file_count for m in models),
        "input_bytes": sum(m.size_bytes for m in models),
        "estimated_tar_bytes": sum(m.estimated_tar_bytes for m in models),
        "chunks": entries,
    }

    if args.dry_run:
        print(json.dumps(manifest, indent=2))
    else:
        manifest_path = output_dir / MANIFEST_NAME
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"[PACK] wrote {manifest_path}")
    return 0


def validate_member(member: tarfile.TarInfo) -> None:
    path = Path(member.name)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe tar member path: {member.name}")
    if not member.name.startswith("data/"):
        raise ValueError(f"archive member is outside data/: {member.name}")


def unpack(args: argparse.Namespace) -> int:
    repo_root = args.repo_root.resolve()
    chunks_dir = (repo_root / args.chunks_dir).resolve()
    manifest_path = chunks_dir / MANIFEST_NAME
    if not manifest_path.exists():
        print(f"error: manifest not found: {manifest_path}", file=sys.stderr)
        return 1

    manifest = json.loads(manifest_path.read_text())
    data_dir = repo_root / manifest.get("data_dir", "data")
    if data_dir.exists() and args.overwrite:
        print(f"[UNPACK] removing existing {data_dir}")
        shutil.rmtree(data_dir)
    elif data_dir.exists() and not args.merge:
        print(f"error: {data_dir} exists; use --overwrite or --merge", file=sys.stderr)
        return 1

    for entry in manifest["chunks"]:
        archive_path = chunks_dir / entry["archive"]
        if not archive_path.exists():
            print(f"error: missing archive: {archive_path}", file=sys.stderr)
            return 1
        if args.verify_sha256:
            actual = file_sha256(archive_path)
            expected = entry.get("sha256")
            if expected and actual != expected:
                print(f"error: sha256 mismatch for {archive_path}", file=sys.stderr)
                print(f"expected {expected}", file=sys.stderr)
                print(f"actual   {actual}", file=sys.stderr)
                return 1
        print(f"[UNPACK] {archive_path}")
        with tarfile.open(archive_path, mode="r:gz") as tar:
            members = tar.getmembers()
            for member in members:
                validate_member(member)
            tar.extractall(repo_root, members=members)
    print(f"[UNPACK] reconstructed {data_dir}")
    return 0


def verify(args: argparse.Namespace) -> int:
    repo_root = args.repo_root.resolve()
    chunks_dir = (repo_root / args.chunks_dir).resolve()
    manifest_path = chunks_dir / MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text())
    errors = 0
    for entry in manifest["chunks"]:
        archive_path = chunks_dir / entry["archive"]
        if not archive_path.exists():
            print(f"[VERIFY] missing {archive_path}")
            errors += 1
            continue
        actual = file_sha256(archive_path)
        expected = entry.get("sha256")
        if expected and actual != expected:
            print(f"[VERIFY] sha256 mismatch {archive_path}")
            errors += 1
        else:
            print(f"[VERIFY] ok {archive_path}")
    return 1 if errors else 0


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=script_dir)

    subparsers = parser.add_subparsers(dest="command", required=True)

    pack_parser = subparsers.add_parser("pack", help="pack data/ into tar.gz chunks")
    pack_parser.add_argument("--data-dir", type=Path, default=Path("data"))
    pack_parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    pack_parser.add_argument("--prefix", default=DEFAULT_PREFIX)
    pack_parser.add_argument("--max-chunk-gb", type=float, default=DEFAULT_MAX_CHUNK_GB)
    pack_parser.add_argument("--compresslevel", type=int, default=6, choices=range(1, 10))
    pack_parser.add_argument("--overwrite", action="store_true")
    pack_parser.add_argument("--dry-run", action="store_true")
    pack_parser.set_defaults(func=pack)

    unpack_parser = subparsers.add_parser("unpack", help="unpack chunks and reconstruct data/")
    unpack_parser.add_argument("--chunks-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    unpack_parser.add_argument("--overwrite", action="store_true")
    unpack_parser.add_argument("--merge", action="store_true")
    unpack_parser.add_argument("--verify-sha256", action="store_true")
    unpack_parser.set_defaults(func=unpack)

    verify_parser = subparsers.add_parser("verify", help="verify chunk checksums")
    verify_parser.add_argument("--chunks-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    verify_parser.set_defaults(func=verify)

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
