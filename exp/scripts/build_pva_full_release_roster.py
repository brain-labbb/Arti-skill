#!/usr/bin/env python3
"""Freeze the complete PV-A release into a resumable source-bound roster."""

from __future__ import annotations

import argparse
import csv
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tempfile
import time
from typing import Any, Iterable, Iterator, Mapping

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import build_table123_full_release_rosters as table123_rosters
import table123_full_release_common as common
from prepare_ours_pva_800 import resolve_archive_name


ROSTER_SCHEMA_VERSION = "pva_table1234_full_release_roster_v1"
SHARD_SCHEMA_VERSION = "pva_table1234_roster_shard_v1"
ARCHIVE_RECEIPT_SCHEMA_VERSION = "pva_archive_receipt_v1"
EXPECTED_RELEASE_COUNT = 302_440
EXPECTED_CATEGORY_COUNT = 531
EXPECTED_MANIFEST_SHA256 = "11bbfa00067e5b8a4fe788db085f896a9754a6f2ec88818c16d9cee1c137c06a"
EXPECTED_MANIFEST_FIELDS = ("slug", "stem", "seed", "asset_id", "overrides_json")
DEFAULT_DATASET_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A")
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "runtime" / "pva_table1234_full_release_20260826" / "roster"


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _atomic_json(path: Path, value: Any) -> None:
    common._atomic_write_bytes(path, _canonical_bytes(value) + b"\n")


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def _write_progress(
    output: Path,
    *,
    status: str,
    completed_categories: int,
    completed_assets: int,
    completed_joints: int,
    total_categories: int,
    total_assets: int,
    started: float,
    current_category: str | None = None,
) -> None:
    _atomic_json(
        output / "progress.json",
        {
            "schema_version": "pva_table1234_roster_progress_v1",
            "status": status,
            "completed_categories": completed_categories,
            "completed_assets": completed_assets,
            "completed_joints": completed_joints,
            "total_categories": total_categories,
            "total_assets": total_assets,
            "current_category": current_category,
            "elapsed_seconds": time.monotonic() - started,
            "updated_at_utc": _utc_now(),
        },
    )


def _safe_component(value: str, field: str) -> str:
    candidate = PurePosixPath(value)
    if (
        not value
        or candidate.is_absolute()
        or len(candidate.parts) != 1
        or value in {".", ".."}
        or "\\" in value
    ):
        raise ValueError(f"invalid PV-A {field}: {value!r}")
    return value


def _load_source_manifest(path: Path) -> list[dict[str, str]]:
    path = Path(path).resolve(strict=True)
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != EXPECTED_MANIFEST_FIELDS:
            raise ValueError(
                f"PV-A manifest schema mismatch: {reader.fieldnames!r}"
            )
        rows = [dict(row) for row in reader]
    if not rows:
        raise ValueError("PV-A manifest is empty")
    seen: set[tuple[str, str]] = set()
    for number, row in enumerate(rows, 2):
        slug = _safe_component(str(row.get("slug", "")), "slug")
        asset_id = _safe_component(str(row.get("asset_id", "")), "asset_id")
        if re.fullmatch(r"seed_\d{4}", asset_id) is None:
            raise ValueError(f"invalid PV-A asset_id at manifest line {number}: {asset_id!r}")
        identity = (slug, asset_id)
        if identity in seen:
            raise ValueError(f"duplicate PV-A identity: {slug}/{asset_id}")
        seen.add(identity)
        try:
            seed = int(str(row.get("seed", "")))
        except ValueError as error:
            raise ValueError(f"invalid PV-A seed at manifest line {number}") from error
        if seed < 0:
            raise ValueError(f"negative PV-A seed at manifest line {number}: {seed}")
        try:
            overrides = json.loads(str(row.get("overrides_json", "")))
        except json.JSONDecodeError as error:
            raise ValueError(f"invalid overrides_json at manifest line {number}") from error
        if not isinstance(overrides, dict):
            raise ValueError(f"overrides_json must be an object at manifest line {number}")
    return rows


def _hash_archive(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"PV-A archive is not a regular file: {path}")
    return {
        "name": path.name,
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": common.sha256_file(path),
    }


def _build_archive_receipt(archives_root: Path, output: Path, workers: int) -> dict[str, Any]:
    paths = sorted(archives_root.glob("*.tar.zst"), key=lambda item: item.name)
    if not paths:
        raise ValueError(f"PV-A archives are missing: {archives_root}")
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        archives = list(executor.map(_hash_archive, paths))
    receipt: dict[str, Any] = {
        "schema_version": ARCHIVE_RECEIPT_SCHEMA_VERSION,
        "archive_count": len(archives),
        "total_bytes": sum(int(item["bytes"]) for item in archives),
        "archives": archives,
    }
    receipt["receipt_content_sha256"] = common.canonical_sha256(receipt)
    _atomic_json(output, receipt)
    return receipt


def _load_archive_receipt(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid PV-A archive receipt: {error}") from error
    if not isinstance(value, dict) or value.get("schema_version") != ARCHIVE_RECEIPT_SCHEMA_VERSION:
        raise ValueError("PV-A archive receipt schema mismatch")
    declared = value.get("receipt_content_sha256")
    if declared != common.canonical_sha256(_without(value, "receipt_content_sha256")):
        raise ValueError("PV-A archive receipt self-hash mismatch")
    archives = value.get("archives")
    if not isinstance(archives, list) or value.get("archive_count") != len(archives):
        raise ValueError("PV-A archive receipt count mismatch")
    return value


def _asset_row(
    source: Mapping[str, str],
    *,
    dataset_root: Path,
    archive_by_name: Mapping[str, Mapping[str, Any]],
    archive_names: set[str] | frozenset[str],
) -> dict[str, Any]:
    slug = str(source["slug"])
    source_asset_id = str(source["asset_id"])
    archive_name = resolve_archive_name(slug, source_asset_id, archive_names)
    package = dataset_root / "extracted" / slug / source_asset_id
    urdf = package / "model.urdf"
    if not package.is_dir() or package.is_symlink():
        raise ValueError(f"PV-A package is missing or symlinked: {slug}/{source_asset_id}")
    for required in ("model.urdf", "appearance.json", "physics.json"):
        candidate = package / required
        if not candidate.is_file() or candidate.is_symlink():
            raise ValueError(f"PV-A package missing {required}: {slug}/{source_asset_id}")
    row = table123_rosters._row(
        urdf,
        source_root=dataset_root,
        asset_id=f"PV-A/{slug}/{source_asset_id}",
        category=slug,
        package=package,
        source_asset_id=source_asset_id,
        slug=slug,
        stem=str(source["stem"]),
        seed=int(source["seed"]),
        overrides_json=str(source["overrides_json"]),
        archive_name=archive_name,
        archive_sha256=str(archive_by_name[archive_name]["sha256"]),
    )
    row["joint_count"] = len(row.get("non_fixed_joints", []))
    row["primary_urdf_size"] = int(row["primary_urdf_bytes"])
    row["release_dataset"] = "PV-A"
    return row


def _shard_receipt_path(shards_root: Path, index: int) -> Path:
    return shards_root / f"{index:04d}.receipt.json"


def _verify_frozen_package(row: Mapping[str, Any]) -> None:
    asset_id = str(row.get("asset_id", ""))
    source_value = row.get("source_path")
    if not isinstance(source_value, str) or not source_value:
        raise ValueError(f"PV-A roster package binding is missing: {asset_id}")
    source = Path(source_value)
    if source.is_symlink():
        raise ValueError(f"PV-A roster package is symlinked: {asset_id}")
    source = source.resolve(strict=True)
    expected_files = row.get("package_files")
    if not isinstance(expected_files, list):
        raise ValueError(f"PV-A package file binding is missing: {asset_id}")
    if common.canonical_sha256(expected_files) != row.get("package_binding_sha256"):
        raise ValueError(f"PV-A package binding self-hash mismatch: {asset_id}")
    expected_normalized = common._normalize_package_files(expected_files)
    observed_files, _observed_hash = common._package_binding(source)
    if observed_files != expected_normalized:
        raise ValueError(f"PV-A package binding drift: {asset_id}")


def _verify_shard(
    path: Path,
    receipt_path: Path,
    *,
    slug: str,
    expected_count: int,
    expected_input_sha256: str | None = None,
    verify_packages: bool = False,
) -> dict[str, Any]:
    try:
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid PV-A roster shard receipt: {error}") from error
    if receipt.get("schema_version") != SHARD_SCHEMA_VERSION:
        raise ValueError("PV-A roster shard schema mismatch")
    declared = receipt.get("receipt_content_sha256")
    if declared != common.canonical_sha256(_without(receipt, "receipt_content_sha256")):
        raise ValueError("PV-A roster shard self-hash mismatch")
    if receipt.get("slug") != slug or int(receipt.get("row_count", -1)) != expected_count:
        raise ValueError(f"PV-A roster shard identity mismatch: {slug}")
    shard_input = receipt.get("shard_input")
    if not isinstance(shard_input, Mapping):
        raise ValueError(f"PV-A roster shard input binding is missing: {slug}")
    declared_input_hash = shard_input.get("shard_input_sha256")
    if declared_input_hash != common.canonical_sha256(
        _without(shard_input, "shard_input_sha256")
    ):
        raise ValueError(f"PV-A roster shard input self-hash mismatch: {slug}")
    if receipt.get("shard_input_sha256") != declared_input_hash:
        raise ValueError(f"PV-A roster shard input receipt mismatch: {slug}")
    if (
        expected_input_sha256 is not None
        and receipt.get("shard_input_sha256") != expected_input_sha256
    ):
        raise ValueError(f"PV-A roster shard input binding mismatch: {slug}")
    if not path.is_file() or common.sha256_file(path) != receipt.get("sha256"):
        raise ValueError(f"PV-A roster shard hash mismatch: {slug}")
    if verify_packages:
        observed_rows = 0
        with path.open(encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(
                        f"invalid PV-A roster shard JSON {slug}:{line_number}"
                    ) from error
                if not isinstance(row, dict) or row.get("raw_category") != slug:
                    raise ValueError(f"PV-A roster shard row mismatch: {slug}:{line_number}")
                _verify_frozen_package(row)
                observed_rows += 1
        if observed_rows != expected_count:
            raise ValueError(f"PV-A roster shard row count drift: {slug}")
    return receipt


def _write_shard(path: Path, rows: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    materialized = list(rows)
    payload = b"".join(_canonical_bytes(dict(row)) + b"\n" for row in materialized)
    common._atomic_write_bytes(path, payload)
    return {
        "row_count": len(materialized),
        "joint_count": sum(int(row.get("joint_count", 0)) for row in materialized),
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def _builder_source_hashes() -> dict[str, str]:
    sources = {
        "pva_roster_builder": Path(__file__).resolve(),
        "table123_roster_builder": Path(table123_rosters.__file__).resolve(),
        "table123_common": Path(common.__file__).resolve(),
        "pva_archive_resolver": SCRIPT_DIR / "prepare_ours_pva_800.py",
    }
    return {name: common.sha256_file(path) for name, path in sorted(sources.items())}


def _shard_input_binding(
    slug: str,
    source_rows: list[dict[str, str]],
    *,
    archive_receipt_sha256: str,
    builder_source_hashes: Mapping[str, str],
) -> dict[str, Any]:
    value: dict[str, Any] = {
        "schema_version": "pva_table1234_roster_shard_input_v1",
        "slug": slug,
        "source_rows_sha256": common.canonical_sha256(source_rows),
        "source_row_count": len(source_rows),
        "archive_receipt_sha256": archive_receipt_sha256,
        "builder_source_hashes": dict(builder_source_hashes),
    }
    value["shard_input_sha256"] = common.canonical_sha256(value)
    return value


def _publish_ordered_roster(
    output: Path,
    categories: list[str],
    grouped: Mapping[str, list[dict[str, str]]],
    shard_inputs: Mapping[str, Mapping[str, Any]],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    roster_path = output / "full_release_roster.jsonl"
    fd, temporary_name = tempfile.mkstemp(prefix=f".{roster_path.name}.", dir=str(output))
    roster_digest = hashlib.sha256()
    identity_digest = hashlib.sha256()
    row_count = joint_count = 0
    shard_entries: list[dict[str, Any]] = []
    try:
        with os.fdopen(fd, "wb") as stream:
            for shard_index, slug in enumerate(categories):
                shard_path = output / "roster_shards" / f"{shard_index:04d}.jsonl"
                receipt_path = _shard_receipt_path(output / "roster_shards", shard_index)
                receipt = _verify_shard(
                    shard_path,
                    receipt_path,
                    slug=slug,
                    expected_count=len(grouped[slug]),
                    expected_input_sha256=str(
                        shard_inputs[slug]["shard_input_sha256"]
                    ),
                )
                shard_entries.append(
                    {
                        "slug": slug,
                        "path": shard_path.relative_to(output).as_posix(),
                        "receipt": receipt_path.relative_to(output).as_posix(),
                        "row_count": int(receipt["row_count"]),
                        "joint_count": int(receipt["joint_count"]),
                        "sha256": str(receipt["sha256"]),
                        "receipt_bytes": receipt_path.stat().st_size,
                        "receipt_sha256": common.sha256_file(receipt_path),
                        "receipt_content_sha256": str(
                            receipt["receipt_content_sha256"]
                        ),
                        "shard_input_sha256": str(receipt["shard_input_sha256"]),
                    }
                )
                for line_number, line in enumerate(shard_path.read_bytes().splitlines(), 1):
                    try:
                        row = json.loads(line)
                    except json.JSONDecodeError as error:
                        raise ValueError(f"invalid roster shard JSON {slug}:{line_number}") from error
                    row["ordinal"] = row_count
                    encoded = _canonical_bytes(row) + b"\n"
                    stream.write(encoded)
                    roster_digest.update(encoded)
                    identity_digest.update(str(row["asset_id"]).encode("utf-8") + b"\n")
                    row_count += 1
                    joint_count += int(row.get("joint_count", 0))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, roster_path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
    return (
        {
            "path": roster_path.relative_to(output).as_posix(),
            "bytes": roster_path.stat().st_size,
            "sha256": roster_digest.hexdigest(),
            "ordered_asset_ids_sha256": identity_digest.hexdigest(),
            "row_count": row_count,
            "joint_count": joint_count,
        },
        shard_entries,
    )


def build_roster(
    dataset_root: Path,
    output: Path,
    *,
    workers: int = 32,
    expected_n: int = EXPECTED_RELEASE_COUNT,
    expected_categories: int = EXPECTED_CATEGORY_COUNT,
    resume: bool = False,
) -> Path:
    if workers <= 0 or expected_n <= 0 or expected_categories <= 0:
        raise ValueError("workers and expected denominators must be positive")
    dataset_root = Path(dataset_root).resolve(strict=True)
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()) and not resume:
        raise ValueError(f"roster output is not empty; use --resume: {output}")
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    source_manifest = dataset_root / "manifest.csv"
    rows = _load_source_manifest(source_manifest)
    source_manifest_sha256 = common.sha256_file(source_manifest)
    categories = sorted({str(row["slug"]) for row in rows})
    if len(rows) != expected_n:
        raise ValueError(f"PV-A release count mismatch: {len(rows)} != {expected_n}")
    if len(categories) != expected_categories:
        raise ValueError(
            f"PV-A category count mismatch: {len(categories)} != {expected_categories}"
        )
    grouped: dict[str, list[dict[str, str]]] = {slug: [] for slug in categories}
    for row in rows:
        grouped[str(row["slug"])].append(row)
    for values in grouped.values():
        values.sort(key=lambda item: str(item["asset_id"]))

    _write_progress(
        output,
        status="HASHING_ARCHIVES",
        completed_categories=0,
        completed_assets=0,
        completed_joints=0,
        total_categories=expected_categories,
        total_assets=expected_n,
        started=started,
    )

    archive_receipt_path = output / "archive_receipt.json"
    archive_receipt = _build_archive_receipt(
        dataset_root / "archives", archive_receipt_path, workers
    )
    archive_by_name = {
        str(item["name"]): item for item in archive_receipt["archives"]
    }
    archive_names = frozenset(archive_by_name)
    for row in rows:
        resolve_archive_name(str(row["slug"]), str(row["asset_id"]), archive_names)

    archive_receipt_sha256 = common.sha256_file(archive_receipt_path)
    builder_source_hashes = _builder_source_hashes()
    shard_inputs = {
        slug: _shard_input_binding(
            slug,
            grouped[slug],
            archive_receipt_sha256=archive_receipt_sha256,
            builder_source_hashes=builder_source_hashes,
        )
        for slug in categories
    }

    shards_root = output / "roster_shards"
    shards_root.mkdir(parents=True, exist_ok=True)
    completed_assets = 0
    completed_joints = 0
    for shard_index, slug in enumerate(categories):
        shard_path = shards_root / f"{shard_index:04d}.jsonl"
        receipt_path = _shard_receipt_path(shards_root, shard_index)
        input_binding = shard_inputs[slug]
        receipt: dict[str, Any] | None = None
        if resume and shard_path.is_file() and receipt_path.is_file():
            try:
                receipt = _verify_shard(
                    shard_path,
                    receipt_path,
                    slug=slug,
                    expected_count=len(grouped[slug]),
                    expected_input_sha256=str(input_binding["shard_input_sha256"]),
                    verify_packages=True,
                )
            except ValueError:
                receipt = None
        if receipt is None:
            builder = lambda source: _asset_row(  # noqa: E731 - executor callback
                source,
                dataset_root=dataset_root,
                archive_by_name=archive_by_name,
                archive_names=archive_names,
            )
            with ThreadPoolExecutor(max_workers=workers) as executor:
                built = list(executor.map(builder, grouped[slug]))
            shard = _write_shard(shard_path, built)
            receipt = {
                "schema_version": SHARD_SCHEMA_VERSION,
                "slug": slug,
                "shard_input": input_binding,
                "shard_input_sha256": input_binding["shard_input_sha256"],
                **shard,
            }
            receipt["receipt_content_sha256"] = common.canonical_sha256(receipt)
            _atomic_json(receipt_path, receipt)
        completed_assets += int(receipt["row_count"])
        completed_joints += int(receipt["joint_count"])
        _write_progress(
            output,
            status="BUILDING_ROSTER",
            completed_categories=shard_index + 1,
            completed_assets=completed_assets,
            completed_joints=completed_joints,
            total_categories=expected_categories,
            total_assets=expected_n,
            started=started,
            current_category=slug,
        )

    roster_artifact, shard_entries = _publish_ordered_roster(
        output, categories, grouped, shard_inputs
    )
    if roster_artifact["row_count"] != expected_n:
        raise ValueError("published PV-A roster count mismatch")
    manifest: dict[str, Any] = {
        "schema_version": ROSTER_SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "dataset_slug": "pva",
        "classification": (
            "FORMAL_FULL_RELEASE"
            if expected_n == EXPECTED_RELEASE_COUNT
            and expected_categories == EXPECTED_CATEGORY_COUNT
            and source_manifest_sha256 == EXPECTED_MANIFEST_SHA256
            else "TEST_FIXTURE"
        ),
        "N_release": expected_n,
        "N_eval": expected_n,
        "J_eval": int(roster_artifact["joint_count"]),
        "release_category_count": expected_categories,
        "denominator_policy": (
            "all manifest.csv rows; evaluator failures and timeouts remain in N/J denominators"
        ),
        "source_bindings": {
            "dataset_root": str(dataset_root),
            "source_manifest": str(source_manifest),
            "source_manifest_bytes": source_manifest.stat().st_size,
            "source_manifest_sha256": source_manifest_sha256,
            "archive_receipt": str(archive_receipt_path),
            "archive_receipt_sha256": archive_receipt_sha256,
            "extracted_root": str((dataset_root / "extracted").resolve(strict=True)),
        },
        "roster": roster_artifact,
        "shards": shard_entries,
        "builder_source_hashes": builder_source_hashes,
    }
    manifest["manifest_content_sha256"] = common.canonical_sha256(manifest)
    manifest_path = output / "roster_manifest.json"
    _atomic_json(manifest_path, manifest)
    # Shards already bind every package file hash and are sealed into the
    # ordered roster. Re-reading every package here doubles full-release I/O;
    # each package is verified again immediately before its evaluation.
    load_roster_manifest(manifest_path, verify_rows=False)
    _write_progress(
        output,
        status="COMPLETE",
        completed_categories=expected_categories,
        completed_assets=expected_n,
        completed_joints=int(roster_artifact["joint_count"]),
        total_categories=expected_categories,
        total_assets=expected_n,
        started=started,
    )
    return manifest_path


def _read_manifest(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid PV-A roster manifest: {error}") from error
    if not isinstance(value, dict) or value.get("schema_version") != ROSTER_SCHEMA_VERSION:
        raise ValueError("PV-A roster manifest schema mismatch")
    declared = value.get("manifest_content_sha256")
    if declared != common.canonical_sha256(_without(value, "manifest_content_sha256")):
        raise ValueError("PV-A roster manifest self-hash mismatch")
    return value


def iter_roster_rows(manifest_path: Path) -> Iterator[dict[str, Any]]:
    manifest_path = Path(manifest_path).resolve(strict=True)
    manifest = _read_manifest(manifest_path)
    roster = manifest.get("roster")
    if not isinstance(roster, Mapping) or not isinstance(roster.get("path"), str):
        raise ValueError("PV-A roster artifact binding is missing")
    rows_path = (manifest_path.parent / str(roster["path"])).resolve(strict=True)
    try:
        rows_path.relative_to(manifest_path.parent)
    except ValueError as error:
        raise ValueError("PV-A roster artifact escapes its root") from error
    with rows_path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise ValueError(f"blank PV-A roster row at line {line_number}")
            try:
                row = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid PV-A roster JSON at line {line_number}") from error
            if not isinstance(row, dict):
                raise ValueError(f"PV-A roster row is not an object at line {line_number}")
            yield row


def _manifest_bound_file(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"PV-A {label} path is missing")
    candidate = root / value
    if candidate.is_symlink():
        raise ValueError(f"PV-A {label} is symlinked: {candidate}")
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as error:
        raise ValueError(f"PV-A {label} path escapes or is missing: {candidate}") from error
    if not resolved.is_file():
        raise ValueError(f"PV-A {label} is not a regular file: {resolved}")
    return resolved


def _verify_manifest_shards(
    manifest_path: Path,
    manifest: Mapping[str, Any],
    source_rows: list[dict[str, str]],
) -> None:
    categories = sorted({str(row["slug"]) for row in source_rows})
    grouped: dict[str, list[dict[str, str]]] = {slug: [] for slug in categories}
    for row in source_rows:
        grouped[str(row["slug"])].append(row)
    for rows in grouped.values():
        rows.sort(key=lambda row: str(row["asset_id"]))

    bindings = manifest.get("source_bindings")
    source_hashes = manifest.get("builder_source_hashes")
    entries = manifest.get("shards")
    if not isinstance(bindings, Mapping) or not isinstance(source_hashes, Mapping):
        raise ValueError("PV-A shard source bindings are missing")
    if not isinstance(entries, list) or len(entries) != len(categories):
        raise ValueError("PV-A shard manifest count mismatch")
    if len(source_rows) != int(manifest.get("N_release", -1)):
        raise ValueError("PV-A shard source row count mismatch")
    if len(categories) != int(manifest.get("release_category_count", -1)):
        raise ValueError("PV-A shard source category count mismatch")

    archive_receipt_sha256 = str(bindings.get("archive_receipt_sha256", ""))
    total_rows = 0
    total_joints = 0
    root = manifest_path.parent
    for slug, entry in zip(categories, entries):
        if not isinstance(entry, Mapping) or entry.get("slug") != slug:
            raise ValueError(f"PV-A shard manifest identity mismatch: {slug}")
        shard_path = _manifest_bound_file(root, entry.get("path"), f"shard {slug}")
        receipt_path = _manifest_bound_file(
            root, entry.get("receipt"), f"shard receipt {slug}"
        )
        expected_input = _shard_input_binding(
            slug,
            grouped[slug],
            archive_receipt_sha256=archive_receipt_sha256,
            builder_source_hashes=source_hashes,
        )
        receipt = _verify_shard(
            shard_path,
            receipt_path,
            slug=slug,
            expected_count=len(grouped[slug]),
            expected_input_sha256=str(expected_input["shard_input_sha256"]),
        )
        if receipt_path.stat().st_size != int(entry.get("receipt_bytes", -1)):
            raise ValueError(f"PV-A shard receipt size mismatch: {slug}")
        if common.sha256_file(receipt_path) != entry.get("receipt_sha256"):
            raise ValueError(f"PV-A shard receipt hash mismatch: {slug}")
        if receipt.get("receipt_content_sha256") != entry.get(
            "receipt_content_sha256"
        ):
            raise ValueError(f"PV-A shard receipt content binding mismatch: {slug}")
        if receipt.get("shard_input_sha256") != entry.get("shard_input_sha256"):
            raise ValueError(f"PV-A shard input manifest binding mismatch: {slug}")
        for field in ("row_count", "joint_count", "sha256"):
            if receipt.get(field) != entry.get(field):
                raise ValueError(f"PV-A shard {field} manifest binding mismatch: {slug}")
        total_rows += int(receipt["row_count"])
        total_joints += int(receipt["joint_count"])

    roster = manifest.get("roster")
    if not isinstance(roster, Mapping):
        raise ValueError("PV-A roster artifact binding is missing")
    if total_rows != int(roster.get("row_count", -1)) or total_rows != int(
        manifest.get("N_eval", -1)
    ):
        raise ValueError("PV-A shard row denominator mismatch")
    if total_joints != int(roster.get("joint_count", -1)) or total_joints != int(
        manifest.get("J_eval", -1)
    ):
        raise ValueError("PV-A shard joint denominator mismatch")


def load_roster_manifest(path: Path, *, verify_rows: bool = False) -> dict[str, Any]:
    path = Path(path).resolve(strict=True)
    manifest = _read_manifest(path)
    roster = manifest.get("roster")
    if not isinstance(roster, Mapping):
        raise ValueError("PV-A roster artifact binding is missing")
    rows_path = (path.parent / str(roster.get("path", ""))).resolve(strict=True)
    if common.sha256_file(rows_path) != roster.get("sha256"):
        raise ValueError("PV-A ordered roster hash mismatch")
    if rows_path.stat().st_size != int(roster.get("bytes", -1)):
        raise ValueError("PV-A ordered roster size mismatch")
    bindings = manifest.get("source_bindings")
    if not isinstance(bindings, Mapping):
        raise ValueError("PV-A source bindings are missing")
    source_manifest = Path(str(bindings.get("source_manifest", ""))).resolve(strict=True)
    if common.sha256_file(source_manifest) != bindings.get("source_manifest_sha256"):
        raise ValueError("PV-A source manifest hash drift")
    source_rows = _load_source_manifest(source_manifest)
    if manifest.get("classification") == "FORMAL_FULL_RELEASE":
        if int(manifest.get("N_release", -1)) != EXPECTED_RELEASE_COUNT:
            raise ValueError("formal PV-A roster release denominator mismatch")
        if int(manifest.get("N_eval", -1)) != EXPECTED_RELEASE_COUNT:
            raise ValueError("formal PV-A roster evaluation denominator mismatch")
        if int(manifest.get("release_category_count", -1)) != EXPECTED_CATEGORY_COUNT:
            raise ValueError("formal PV-A roster category denominator mismatch")
        if bindings.get("source_manifest_sha256") != EXPECTED_MANIFEST_SHA256:
            raise ValueError("formal PV-A roster manifest identity mismatch")
    archive_receipt = Path(str(bindings.get("archive_receipt", ""))).resolve(strict=True)
    if common.sha256_file(archive_receipt) != bindings.get("archive_receipt_sha256"):
        raise ValueError("PV-A archive receipt hash drift")
    _load_archive_receipt(archive_receipt)
    _verify_manifest_shards(path, manifest, source_rows)
    if verify_rows:
        ids: set[str] = set()
        categories: set[str] = set()
        joint_count = 0
        identity_digest = hashlib.sha256()
        count = 0
        for expected_ordinal, row in enumerate(iter_roster_rows(path)):
            if row.get("ordinal") != expected_ordinal:
                raise ValueError("PV-A roster ordinals are not contiguous")
            asset_id = str(row.get("asset_id", ""))
            if not asset_id or asset_id in ids:
                raise ValueError(f"duplicate or empty PV-A roster identity: {asset_id!r}")
            ids.add(asset_id)
            categories.add(str(row.get("raw_category", row.get("category", ""))))
            joint_count += int(row.get("joint_count", -1))
            identity_digest.update(asset_id.encode("utf-8") + b"\n")
            primary = Path(str(row.get("primary_urdf_path", "")))
            package = Path(str(row.get("source_path", "")))
            if not primary.is_file() or primary.is_symlink() or not package.is_dir():
                raise ValueError(f"PV-A roster source is unavailable: {asset_id}")
            try:
                primary.resolve().relative_to(package.resolve())
            except ValueError as error:
                raise ValueError(f"PV-A primary URDF escapes package: {asset_id}") from error
            if common.sha256_file(primary) != row.get("primary_urdf_sha256"):
                raise ValueError(f"PV-A primary URDF hash drift: {asset_id}")
            _verify_frozen_package(row)
            count += 1
        if count != int(manifest.get("N_eval", -1)):
            raise ValueError("PV-A roster row count mismatch")
        if joint_count != int(manifest.get("J_eval", -1)):
            raise ValueError("PV-A roster joint count mismatch")
        if len(categories) != int(manifest.get("release_category_count", -1)):
            raise ValueError("PV-A roster category count mismatch")
        if identity_digest.hexdigest() != roster.get("ordered_asset_ids_sha256"):
            raise ValueError("PV-A roster identity-order hash mismatch")
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=32)
    parser.add_argument("--expected-n", type=int, default=EXPECTED_RELEASE_COUNT)
    parser.add_argument("--expected-categories", type=int, default=EXPECTED_CATEGORY_COUNT)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = build_roster(
            args.dataset_root,
            args.output,
            workers=args.workers,
            expected_n=args.expected_n,
            expected_categories=args.expected_categories,
            resume=args.resume,
        )
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "COMPLETE", "roster": str(result)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "EXPECTED_CATEGORY_COUNT",
    "EXPECTED_RELEASE_COUNT",
    "ROSTER_SCHEMA_VERSION",
    "build_roster",
    "iter_roster_rows",
    "load_roster_manifest",
]
