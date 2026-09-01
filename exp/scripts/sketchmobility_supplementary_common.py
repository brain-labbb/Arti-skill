#!/usr/bin/env python3
"""Shared frozen-input contracts for SketchMobility supplementary tables."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = Path(os.environ.get("SKETCHMOBILITY_REPO_ROOT", SCRIPT.parents[2])).resolve()
DEFAULT_DATASET_ROOT = REPO / "exp/SketchMobility"
DEFAULT_TABLE2_RECEIPT = (
    REPO
    / "exp/runtime/table2_urdf_sketch_mobility_table1cohort_n800_20260821T035015Z"
)
DEFAULT_TABLE3_RECEIPT = (
    REPO
    / "exp/runtime/urdf_table3_sketch_mobility_table1cohort_n800_20260821T062050Z"
)
DEFAULT_TABLE4_RECEIPT = (
    REPO
    / "exp/runtime/urdf_table4_sketch_mobility_table1cohort_n800_20260821T090554Z"
)

FORMAL_N_EVAL = 800
FORMAL_J_EVAL = 1824
EXPECTED_TABLE4_MANIFEST_SHA256 = (
    "71b895dea4c9ce220825928a89205a05cb7875e2bfce6372687a52eee596de17"
)
EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256 = (
    "a4dd5597daa79170880f517d0e0058cd58f8828037ff14b0ae9e5515cf967904"
)
EXPECTED_ORDERED_ASSET_IDS_SHA256 = (
    "a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def receipt_file_rows(root: Path) -> list[dict[str, Any]]:
    """Hash every regular evidence file except the two closure files."""
    root = root.resolve(strict=True)
    rows: list[dict[str, Any]] = []
    excluded = {"artifact_manifest.json", "receipt_digest.json"}
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if relative in excluded:
            continue
        rows.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    return rows


def write_receipt_closure(root: Path, atomic_write_json: Any) -> None:
    """Write a non-circular whole-receipt artifact manifest and digest."""
    rows = receipt_file_rows(root)
    artifact = {
        "schema_version": "sketchmobility-artifact-manifest/v1",
        "file_count": len(rows),
        "total_bytes": sum(int(row["bytes"]) for row in rows),
        "files": rows,
        "files_sha256": canonical_sha256(rows),
    }
    atomic_write_json(root / "artifact_manifest.json", artifact)
    receipt = {
        "schema_version": "sketchmobility-receipt-digest/v1",
        "artifact_manifest_sha256": sha256_file(root / "artifact_manifest.json"),
        "files_sha256": artifact["files_sha256"],
    }
    atomic_write_json(root / "receipt_digest.json", receipt)


def validate_receipt_closure(root: Path) -> bool:
    """Recompute the whole receipt and reject additions, removals, or edits."""
    try:
        artifact_path = root / "artifact_manifest.json"
        receipt_path = root / "receipt_digest.json"
        artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        rows = receipt_file_rows(root)
        expected_artifact = {
            "schema_version": "sketchmobility-artifact-manifest/v1",
            "file_count": len(rows),
            "total_bytes": sum(int(row["bytes"]) for row in rows),
            "files": rows,
            "files_sha256": canonical_sha256(rows),
        }
        return (
            artifact == expected_artifact
            and receipt
            == {
                "schema_version": "sketchmobility-receipt-digest/v1",
                "artifact_manifest_sha256": sha256_file(artifact_path),
                "files_sha256": expected_artifact["files_sha256"],
            }
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return False


def _manifest_self_hash(manifest: dict[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def load_frozen_cohort(
    receipt: Path = DEFAULT_TABLE4_RECEIPT,
    *,
    formal: bool,
) -> dict[str, Any]:
    """Load the exact Table 4 manifest and reject membership or order drift."""
    receipt = receipt.resolve(strict=True)
    manifest_path = receipt / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != FORMAL_N_EVAL:
        raise ValueError("frozen cohort must contain exactly 800 items")
    if manifest.get("dataset") != "SketchMobility":
        raise ValueError("frozen cohort dataset mismatch")
    if manifest.get("classification") != "FORMAL":
        raise ValueError("frozen cohort is not classified FORMAL")

    ids: list[str] = []
    joint_count = 0
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f"frozen cohort item {index} is not an object")
        if int(item.get("order", -1)) != index:
            raise ValueError(f"frozen cohort order drift at index {index}")
        if int(item.get("selection_rank", -1)) != index + 1:
            raise ValueError(f"frozen selection rank order drift at index {index}")
        asset_id = str(item.get("asset_id", ""))
        if not asset_id or str(item.get("dataset_id", "")) != asset_id:
            raise ValueError(f"frozen asset identity mismatch at index {index}")
        ids.append(asset_id)
        joint_count += int(item.get("movable_dof_count", -1))

    ordered_hash = canonical_sha256(ids)
    recorded_ordered_hash = (
        manifest.get("selection", {}).get("selected_asset_ids_sha256")
    )
    if ordered_hash != EXPECTED_ORDERED_ASSET_IDS_SHA256:
        raise ValueError(f"ordered asset IDs drift: {ordered_hash}")
    if recorded_ordered_hash != ordered_hash:
        raise ValueError("manifest ordered asset IDs binding mismatch")
    if len(set(ids)) != FORMAL_N_EVAL:
        raise ValueError("frozen cohort asset IDs are not unique")
    if joint_count != FORMAL_J_EVAL:
        raise ValueError(f"frozen joint denominator drift: {joint_count}")
    if manifest.get("manifest_content_sha256") != _manifest_self_hash(manifest):
        raise ValueError("Table 4 manifest self-hash mismatch")
    if formal:
        observed_file_hash = sha256_file(manifest_path)
        if observed_file_hash != EXPECTED_TABLE4_MANIFEST_SHA256:
            raise ValueError(f"formal Table 4 manifest byte drift: {observed_file_hash}")
        if (
            manifest.get("manifest_content_sha256")
            != EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256
        ):
            raise ValueError("formal Table 4 manifest content drift")
        if receipt != DEFAULT_TABLE4_RECEIPT.resolve(strict=True):
            raise ValueError("formal Table 4 receipt path differs from frozen path")

    return {
        "rows": items,
        "joint_count": joint_count,
        "ordered_asset_ids_sha256": ordered_hash,
        "manifest": manifest,
        "manifest_path": str(manifest_path),
        "manifest_file_sha256": sha256_file(manifest_path),
        "manifest_content_sha256": manifest["manifest_content_sha256"],
    }


def _safe_package(dataset_root: Path, asset_id: str) -> Path:
    relative = PurePosixPath(asset_id)
    if (
        relative.is_absolute()
        or len(relative.parts) != 4
        or relative.parts[0] != "data"
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise ValueError(f"invalid SketchMobility asset ID: {asset_id!r}")
    cursor = dataset_root
    for part in relative.parts:
        cursor = cursor / part
        if cursor.is_symlink():
            raise ValueError(f"asset package path contains a symlink: {asset_id}")
    package = cursor.resolve(strict=True)
    package.relative_to(dataset_root)
    if not package.is_dir():
        raise ValueError(f"asset package is not a directory: {asset_id}")
    return package


def package_binding(package: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(
        package, followlinks=False
    ):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ValueError(f"package contains directory symlink: {current / name}")
        for name in file_names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            canonical = path.resolve(strict=True)
            canonical.relative_to(package)
            if not canonical.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            files.append(
                {
                    "path": relative,
                    "bytes": canonical.stat().st_size,
                    "sha256": sha256_file(canonical),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def audit_package(row: dict[str, Any], dataset_root: Path) -> dict[str, Any]:
    dataset_root = dataset_root.resolve(strict=True)
    asset_id = str(row.get("asset_id", ""))
    package = _safe_package(dataset_root, asset_id)
    binding = package_binding(package)
    expected_binding = str(row.get("package_content_manifest_sha256", ""))
    if binding["content_manifest_sha256"] != expected_binding:
        raise ValueError(f"full package binding drift: {asset_id}")
    urdf_path = package / "mobility.urdf"
    if urdf_path.is_symlink() or not urdf_path.is_file():
        raise ValueError(f"mobility.urdf is missing or symlinked: {asset_id}")
    urdf_sha256 = sha256_file(urdf_path)
    if urdf_sha256 != str(row.get("urdf_sha256", "")):
        raise ValueError(f"mobility.urdf binding drift: {asset_id}")
    return {
        "asset_id": asset_id,
        "package": str(package),
        "urdf_path": str(urdf_path.resolve(strict=True)),
        "urdf_sha256": urdf_sha256,
        "package_content_manifest_sha256": binding["content_manifest_sha256"],
        "package_file_count": binding["file_count"],
        "package_total_bytes": binding["total_bytes"],
        "package_binding": binding,
    }
