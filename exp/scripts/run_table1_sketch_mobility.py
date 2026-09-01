#!/usr/bin/env python3
"""Evaluate Table 1 on a frozen SketchMobility N=800 cohort.

SketchMobility (``Arlo397/SketchMobility``, companion release of Sketch2Arti,
arXiv:2604.25781) is a mixed-license secondary curation of four upstream
sources. After unpacking, each object has the release layout::

    data/{source}/{category}/{display_id}/
        annotation.json
        mobility.urdf
        meshes/

This runner produces the first frozen SketchMobility cohort:

- a release roster over all objects declared by
  ``dataset_chunks/manifest.json``, runtime-verified against the unpacked
  tree (directory closure, no symlinks, required files, per-object file
  count and byte count match, chunk-level SHA-256 re-verification);
- a deterministic hash-ranked N=800 sample (no replacement, no
  outcome-based filtering);
- declared-layer Table 1 structural statistics using the shared
  ``run_table1_artiverse.py`` metric core: links/joints XML counts,
  multi-joint share, rooted-tree topology hash
  (``rooted-joint-tree-v1``) and simulation-package fingerprint
  (``simulation-package-fingerprint-v2``) with the object directory as the
  containment root (``mobility.urdf`` + ``meshes/`` closure is natively
  package-contained).

Category policy: the declared category unit is the release pair
``{source}/{category}`` (exact strings, no semantic merging), because
SketchMobility keeps the four upstream category schemes separate and 22 of
47 bare category names collide across sources. Bare category-name counts are
reported as a supplementary diagnostic only.
"""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_DATASET_ROOT = REPO / "exp/SketchMobility"
DEFAULT_OUTPUT = REPO / "exp/runtime/table1_sketch_mobility"
DEFAULT_PROTOCOL = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
SELECTION_PROTOCOL = "sketch-mobility-table1-global-sample-v1"
DEFAULT_SEED = "arti-skill-table1-sketch-mobility-n800-v1"
EXPECTED_RELEASE_OBJECT_COUNT = 4956
KNOWN_SOURCES = ("Agentic", "Infinigen", "PartNeXt", "Shape2Motion")
EXPECTED_SOURCE_COUNTS = {
    "Agentic": 205,
    "Infinigen": 726,
    "PartNeXt": 2177,
    "Shape2Motion": 1848,
}
LICENSE_COMPLIANCE_NOTE = (
    "SketchMobility is a mixed-license compilation (CC BY 4.0 and GPL-3.0 "
    "components per LICENSE.md / LICENSE_MAP.json). This evaluation performs "
    "local read-only structural analysis, caches only derived aggregate "
    "statistics and fingerprints, and does not redistribute asset bytes or "
    "create derivative asset works; aggregate result publication is not "
    "restricted by the GPL-3.0 components. Confirmed acceptable for unified "
    "evaluation execution, caching and result distribution before freeze."
)
UNPACK_METHOD_NOTE = (
    "Chunks were extracted with three parallel native `tar -xzf` processes "
    "(one per chunk; object directories are disjoint across chunks per "
    "dataset_chunks/manifest.json) instead of the sequential release unpack "
    "tool, for throughput on the shared filesystem. Archives contain only "
    "regular files and directories, so extraction is byte-identical. Chunk "
    "sizes and SHA-256 checksums are re-verified by this runner at run time, "
    "and every release object is runtime-verified against its manifest "
    "record (closure, required files, no symlinks, file count, byte count)."
)


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


SHARED = _load_module(
    Path(__file__).with_name("run_table1_artiverse.py"),
    "table1_structural_metrics_shared",
)


def _error_text(error: BaseException) -> str:
    return f"{type(error).__name__}: {error}"


def _reject_package_symlinks(package: Path) -> None:
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {path.relative_to(package)}"
                )
        for name in file_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(
                    f"package contains file symlink: {path.relative_to(package)}"
                )


def _walk_files(package: Path) -> tuple[int, int]:
    file_count = 0
    total_bytes = 0
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in file_names:
            path = current / name
            file_count += 1
            total_bytes += path.stat().st_size
    return file_count, total_bytes


def load_release_manifest(dataset_root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    manifest_path = dataset_root / "dataset_chunks/manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schemaVersion") != 1:
        raise ValueError(f"unexpected release manifest schema: {manifest.get('schemaVersion')!r}")
    if manifest.get("dataset") != "SketchMobility":
        raise ValueError(f"unexpected release dataset name: {manifest.get('dataset')!r}")
    if manifest.get("archiveFormat") != "tar.gz" or manifest.get("archiveRoot") != "data":
        raise ValueError("unexpected release archive format or root")
    chunks = manifest.get("chunks")
    if not isinstance(chunks, list) or not chunks:
        raise ValueError("release manifest contains no chunk records")
    objects = manifest.get("objects")
    if not isinstance(objects, list) or len(objects) != int(manifest.get("objectCount") or -1):
        raise ValueError("release manifest object count mismatch")
    if len(objects) != EXPECTED_RELEASE_OBJECT_COUNT:
        raise ValueError(
            f"expected {EXPECTED_RELEASE_OBJECT_COUNT} release objects, found {len(objects)}"
        )

    identities: list[dict[str, Any]] = []
    for record in objects:
        path = str(record.get("path") or "")
        parts = Path(path).parts
        if (
            len(parts) != 4
            or parts[0] != "data"
            or Path(path).is_absolute()
            or parts[1] not in KNOWN_SOURCES
        ):
            raise ValueError(f"invalid release object path: {path!r}")
        identities.append(
            {
                "asset_id": path,
                "source": parts[1],
                "category": parts[2],
                "source_category": f"{parts[1]}/{parts[2]}",
                "display_id": parts[3],
                "chunk": str(record.get("chunk") or ""),
                "declared_file_count": int(record.get("fileCount") or -1),
                "declared_uncompressed_bytes": int(record.get("uncompressedBytes") or -1),
            }
        )
    if len({row["asset_id"] for row in identities}) != len(identities):
        raise ValueError("release manifest contains duplicate object paths")
    source_counts = Counter(row["source"] for row in identities)
    if dict(source_counts) != EXPECTED_SOURCE_COUNTS:
        raise ValueError(f"unexpected release source composition: {dict(source_counts)}")
    return manifest, identities


def verify_chunks(dataset_root: Path, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for chunk in manifest["chunks"]:
        filename = str(chunk.get("filename") or "")
        if Path(filename).name != filename:
            raise ValueError(f"unsafe chunk filename: {filename!r}")
        path = dataset_root / "dataset_chunks" / filename
        if not path.is_file():
            raise FileNotFoundError(f"missing chunk: {path}")
        size_ok = path.stat().st_size == int(chunk.get("sizeBytes") or -1)
        digest = SHARED.sha256_file(path)
        sha_ok = digest == str(chunk.get("sha256") or "")
        if not (size_ok and sha_ok):
            raise ValueError(f"chunk verification failed: {filename}")
        records.append(
            {
                "filename": filename,
                "size_bytes": path.stat().st_size,
                "sha256": digest,
                "size_verified": size_ok,
                "sha256_verified": sha_ok,
            }
        )
    return records


def build_release_roster(
    dataset_root: Path,
    identities: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for identity in identities:
        row: dict[str, Any] = {
            key: identity[key]
            for key in (
                "asset_id",
                "source",
                "category",
                "source_category",
                "display_id",
                "chunk",
                "declared_file_count",
                "declared_uncompressed_bytes",
            )
        }
        row["roster_status"] = "COMPLETE"
        row["roster_error"] = None
        package = (dataset_root / identity["asset_id"]).resolve(strict=False)
        try:
            package.relative_to(dataset_root.resolve())
        except ValueError:
            row["roster_status"] = "INVALID_OBJECT_PATH"
            row["roster_error"] = "object path escapes dataset root"
            rows.append(row)
            continue
        if not package.is_dir():
            row["roster_status"] = "MISSING_OBJECT_DIRECTORY"
            row["roster_error"] = "object directory is missing after unpack"
            rows.append(row)
            continue
        try:
            _reject_package_symlinks(package)
        except ValueError as error:
            row["roster_status"] = "SYMLINK_REJECTED"
            row["roster_error"] = _error_text(error)
            rows.append(row)
            continue
        urdf = package / "mobility.urdf"
        annotation = package / "annotation.json"
        top_level_urdfs = sorted(
            path.name for path in package.iterdir()
            if path.suffix.lower() == ".urdf" and path.is_file()
        )
        if top_level_urdfs != ["mobility.urdf"]:
            row["roster_status"] = "UNEXPECTED_TOP_LEVEL_URDFS"
            row["roster_error"] = f"top-level urdf set: {top_level_urdfs}"
            rows.append(row)
            continue
        if not urdf.is_file() or not annotation.is_file():
            row["roster_status"] = "MISSING_REQUIRED_FILES"
            row["roster_error"] = "annotation.json or mobility.urdf missing"
            rows.append(row)
            continue
        try:
            file_count, total_bytes = _walk_files(package)
            row["mobility_urdf_sha256"] = SHARED.sha256_file(urdf)
            row["annotation_sha256"] = SHARED.sha256_file(annotation)
        except OSError as error:
            row["roster_status"] = "READ_FAILED"
            row["roster_error"] = _error_text(error)
            rows.append(row)
            continue
        row["runtime_file_count"] = file_count
        row["runtime_uncompressed_bytes"] = total_bytes
        if file_count != identity["declared_file_count"]:
            row["roster_status"] = "FILE_COUNT_MISMATCH"
            row["roster_error"] = (
                f"declared {identity['declared_file_count']} files, found {file_count}"
            )
        elif total_bytes != identity["declared_uncompressed_bytes"]:
            row["roster_status"] = "BYTE_COUNT_MISMATCH"
            row["roster_error"] = (
                f"declared {identity['declared_uncompressed_bytes']} bytes, "
                f"found {total_bytes}"
            )
        rows.append(row)
    return rows


def freeze_selection(
    identities: list[dict[str, Any]],
    *,
    sample_size: int,
    seed: str,
    release_manifest_sha256: str,
) -> list[dict[str, Any]]:
    if sample_size <= 0:
        raise ValueError("sample size must be positive")
    if sample_size > len(identities):
        raise ValueError(
            f"sample size {sample_size} exceeds roster size {len(identities)}"
        )
    asset_ids = [str(row["asset_id"]) for row in identities]
    if len(set(asset_ids)) != len(asset_ids):
        raise ValueError("roster identities are not unique")

    ranked: list[dict[str, Any]] = []
    for row in identities:
        asset_id = str(row["asset_id"])
        payload = "\0".join(
            (SELECTION_PROTOCOL, release_manifest_sha256, str(seed), asset_id)
        ).encode("utf-8")
        ranked.append({**row, "selection_hash": hashlib.sha256(payload).hexdigest()})
    ranked.sort(key=lambda row: (row["selection_hash"], row["asset_id"]))
    return [
        {**row, "selection_rank": rank}
        for rank, row in enumerate(ranked[:sample_size], start=1)
    ]


def _annotation_summary(annotation_path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(annotation_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return {"annotation_parse_success": False, "annotation_error": _error_text(error)}
    if not isinstance(payload, dict):
        return {"annotation_parse_success": False, "annotation_error": "not a JSON object"}

    def pick(*names: str) -> Any:
        for name in names:
            if name in payload:
                return payload[name]
        return None

    joints = pick("joints", "jointList", "joint_annotations")
    return {
        "annotation_parse_success": True,
        "annotation_error": None,
        "annotation_keys": sorted(payload.keys()),
        "annotation_upstream_id": pick("upstreamId", "upstream_id"),
        "annotation_release_id": pick("releaseId", "release_id", "displayId", "display_id"),
        "annotation_source": pick("sourceDataset", "source", "sourceLabel"),
        "annotation_category": pick("category"),
        "annotation_declared_urdf": pick("urdf"),
        "annotation_joint_count": len(joints) if isinstance(joints, list) else None,
    }


def evaluate_asset(dataset_root: Path, identity: dict[str, Any]) -> dict[str, Any]:
    record: dict[str, Any] = {
        **{
            key: identity[key]
            for key in (
                "asset_id",
                "source",
                "category",
                "source_category",
                "display_id",
                "selection_rank",
                "selection_hash",
            )
        },
        "raw_category": identity["source_category"],
        "status": None,
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
        "runtime_binding_ok": False,
        "primary_urdf": None,
        "primary_urdf_sha256": None,
        "error": None,
    }
    package = (dataset_root / identity["asset_id"]).resolve(strict=False)
    try:
        package.relative_to(dataset_root.resolve())
    except ValueError:
        record["status"] = "INVALID_OBJECT_PATH"
        record["error"] = "object path escapes dataset root"
        return record
    if not package.is_dir():
        record["status"] = "MISSING_OBJECT_DIRECTORY"
        record["error"] = "object directory is missing"
        return record
    try:
        _reject_package_symlinks(package)
    except ValueError as error:
        record["status"] = "SYMLINK_REJECTED"
        record["error"] = _error_text(error)
        return record

    urdf_path = package / "mobility.urdf"
    top_level_urdfs = sorted(
        path.name for path in package.iterdir()
        if path.suffix.lower() == ".urdf" and path.is_file()
    )
    if top_level_urdfs != ["mobility.urdf"]:
        record["status"] = "AMBIGUOUS_PRIMARY_URDF"
        record["error"] = f"top-level urdf set: {top_level_urdfs}"
        return record
    record["primary_urdf"] = (
        urdf_path.relative_to(dataset_root).as_posix()
    )
    try:
        record["primary_urdf_sha256"] = SHARED.sha256_file(urdf_path)
    except OSError as error:
        record["status"] = "URDF_READ_FAILED"
        record["error"] = _error_text(error)
        return record

    try:
        file_count, total_bytes = _walk_files(package)
        record["runtime_file_count"] = file_count
        record["runtime_uncompressed_bytes"] = total_bytes
        record["runtime_binding_ok"] = (
            file_count == identity["declared_file_count"]
            and total_bytes == identity["declared_uncompressed_bytes"]
        )
    except OSError as error:
        record["status"] = "RUNTIME_BINDING_FAILED"
        record["error"] = _error_text(error)
        return record
    if not record["runtime_binding_ok"]:
        record["status"] = "RUNTIME_BINDING_MISMATCH"
        record["error"] = (
            f"declared ({identity['declared_file_count']} files, "
            f"{identity['declared_uncompressed_bytes']} bytes) != runtime "
            f"({record['runtime_file_count']} files, "
            f"{record['runtime_uncompressed_bytes']} bytes)"
        )
        return record

    record.update(_annotation_summary(package / "annotation.json"))

    try:
        analysis = SHARED.analyze_urdf(urdf_path)
    except (ET.ParseError, OSError, ValueError) as error:
        record["status"] = "URDF_PARSE_FAILED"
        record["error"] = _error_text(error)
        return record
    record.update(analysis)
    record["parse_success"] = True

    try:
        fingerprint = SHARED.fingerprint_package(urdf_path)
    except (ET.ParseError, OSError, ValueError) as error:
        record["status"] = "FINGERPRINT_FAILED"
        record["error"] = _error_text(error)
        return record
    record["fingerprint_complete"] = bool(fingerprint["complete"])
    record["package_fingerprint"] = fingerprint["fingerprint"]
    record["referenced_resource_count"] = fingerprint["resource_count"]
    record["missing_resources"] = fingerprint["missing_resources"]
    if fingerprint["complete"]:
        record["status"] = "EVALUATED"
    else:
        record["status"] = "EVALUATED_FINGERPRINT_INCOMPLETE"
        record["error"] = "one or more referenced package resources are unavailable"
    return record


def _evaluate_asset_fail_closed(dataset_root: Path, identity: dict[str, Any]) -> dict[str, Any]:
    try:
        return evaluate_asset(dataset_root, identity)
    except Exception as error:  # noqa: BLE001 - fail-closed by design
        return {
            "asset_id": identity.get("asset_id"),
            "source": identity.get("source"),
            "category": identity.get("category"),
            "source_category": identity.get("source_category"),
            "display_id": identity.get("display_id"),
            "raw_category": identity.get("source_category"),
            "selection_rank": identity.get("selection_rank"),
            "selection_hash": identity.get("selection_hash"),
            "status": "ASSET_EVALUATION_FAILED",
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
            "runtime_binding_ok": False,
            "primary_urdf": None,
            "primary_urdf_sha256": None,
            "error": _error_text(error),
        }


def _source_breakdown(records: list[dict[str, Any]]) -> dict[str, Any]:
    breakdown: dict[str, Any] = {}
    for source in KNOWN_SOURCES:
        rows = [row for row in records if row.get("source") == source]
        parsed = [row for row in rows if row.get("parse_success")]
        movable = [int(row["non_fixed_joint_count"]) for row in parsed]
        topology_hashes = [
            str(row["topology_hash"])
            for row in rows
            if row.get("valid_tree") and row.get("topology_hash")
        ]
        fingerprints = [
            str(row["package_fingerprint"])
            for row in rows
            if row.get("fingerprint_complete") and row.get("package_fingerprint")
        ]
        fingerprint_counts = Counter(fingerprints)
        multi_joint = sum(value >= 2 for value in movable)
        breakdown[source] = {
            "N_eval": len(rows),
            "N_parse": len(parsed),
            "links_per_asset_mean": (
                sum(int(row["link_count"]) for row in parsed) / len(parsed)
                if parsed
                else None
            ),
            "movable_joints_per_asset_mean": (
                sum(movable) / len(parsed) if parsed else None
            ),
            "multi_joint_assets_rate": (
                multi_joint / len(rows) if rows else None
            ),
            "unique_topologies_rate": (
                len(set(topology_hashes)) / len(topology_hashes)
                if topology_hashes
                else None
            ),
            "exact_duplicate_rate": (
                (len(fingerprints) - len(fingerprint_counts)) / len(fingerprints)
                if fingerprints
                else None
            ),
        }
    return breakdown


def aggregate_records(
    records: list[dict[str, Any]],
    *,
    release_asset_count: int,
    release_source_category_count: int,
    release_category_name_count: int,
) -> dict[str, Any]:
    summary = SHARED.aggregate_records(
        records,
        release_asset_count=release_asset_count,
        release_category_count=release_source_category_count,
    )
    summary["cohort"]["cohort_type"] = "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED"
    summary["cohort"]["release_category_name_count"] = release_category_name_count
    summary["cohort"]["eval_category_name_count"] = len(
        {str(row["category"]) for row in records}
    )
    summary["cohort"]["category_policy"] = (
        "declared release pair {source}/{category}, exact strings, no semantic "
        "merging; bare category names are supplementary diagnostics only"
    )
    summary["source_breakdown"] = _source_breakdown(records)
    return summary


def _report(summary: dict[str, Any], run_manifest: dict[str, Any]) -> str:
    cohort = summary["cohort"]
    links = summary["links_per_asset"]
    movable = summary["movable_joints_per_asset"]
    multi = summary["multi_joint_assets"]
    topology = summary["unique_topologies"]
    duplicate = summary["exact_duplicate_rate"]

    def percentage(value: float | None) -> str:
        return "N/E" if value is None else f"{100.0 * value:.2f}%"

    def statistic(value: float | int | None, *, decimals: int | None = None) -> str:
        if value is None:
            return "N/E"
        if decimals is None:
            return str(value)
        return f"{value:.{decimals}f}"

    lines = [
        "# SketchMobility Table 1: Dataset Scale and Structural Diversity",
        "",
        "## Frozen cohort",
        "",
        f"- Release: `Arlo397/SketchMobility` (Sketch2Arti companion release).",
        f"- N_release: {cohort['N_release']}",
        f"- N_eval: {cohort['N_eval']} (parsed: {cohort['N_parse']})",
        f"- Selection protocol: `{run_manifest['selection_protocol']}`",
        f"- Selection policy: {run_manifest['selection_policy']}",
        f"- Seed: `{run_manifest['seed']}`",
        f"- Cohort type: {cohort['cohort_type']}",
        f"- Release categories ({cohort['category_policy'].split(';')[0]}): "
        f"{cohort['release_raw_categories']}; sample: {cohort['eval_raw_categories']}",
        f"- Supplementary bare category names: release "
        f"{cohort['release_category_name_count']} / sample "
        f"{cohort['eval_category_name_count']}",
        "",
        "## Headline metrics (sample)",
        "",
        "| Metric | Value |",
        "|---|---|",
        f"| Links/Asset (mean / median / P90) | {statistic(links['mean'], decimals=2)} / "
        f"{statistic(links['median'], decimals=0)} / {statistic(links['p90_nearest_rank'])} |",
        f"| Movable Joints/Asset (mean / median / P90) | "
        f"{statistic(movable['mean'], decimals=2)} / "
        f"{statistic(movable['median'], decimals=0)} / "
        f"{statistic(movable['p90_nearest_rank'])} |",
        f"| Multi-joint Assets | {multi['numerator']} / {multi['denominator']} "
        f"({percentage(multi['rate'])}) |",
        f"| Unique Topologies | {topology['unique']} / {topology['denominator']} "
        f"({percentage(topology['rate'])}); coverage {topology['coverage_denominator']} "
        f"evaluable of N_eval ({percentage(topology['coverage_rate'])}) |",
        f"| Exact Duplicate Rate | {duplicate['duplicate_excess']} / "
        f"{duplicate['denominator']} ({percentage(duplicate['rate'])}); coverage "
        f"{duplicate['denominator']} fingerprint-complete of N_eval "
        f"{duplicate['coverage_denominator']} ({percentage(duplicate['coverage_rate'])}) |",
        "",
        "## Declared joint type counts (sample, parsed assets)",
        "",
    ]
    for joint_type, count in sorted((summary.get("declared_joint_type_counts") or {}).items()):
        lines.append(f"- `{joint_type}`: {count}")
    lines.extend(
        [
            "",
            "## Source breakdown (sample)",
            "",
            "| Source | N_eval | N_parse | Multi-joint | Unique Topologies | Exact Duplicate Rate |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for source, row in sorted(summary["source_breakdown"].items()):
        lines.append(
            f"| {source} | {row['N_eval']} | {row['N_parse']} | "
            f"{percentage(row['multi_joint_assets_rate'])} | "
            f"{percentage(row['unique_topologies_rate'])} | "
            f"{percentage(row['exact_duplicate_rate'])} |"
        )
    lines.extend(
        [
            "",
            "## Status counts",
            "",
        ]
    )
    for status, count in sorted((summary.get("status_counts") or {}).items()):
        lines.append(f"- `{status}`: {count}")
    lines.extend(
        [
            "",
            "## Provenance",
            "",
            f"- Runner: `{run_manifest['runner']}` "
            f"(sha256 `{run_manifest['runner_sha256']}`)",
            f"- Shared metric core: `{run_manifest['shared_metric_runner']}` "
            f"(sha256 `{run_manifest['shared_metric_runner_sha256']}`)",
            f"- Protocol document sha256: `{run_manifest['protocol_sha256']}`",
            f"- Release manifest sha256: `{run_manifest['release_manifest_sha256']}`",
            f"- Release roster sha256: `{run_manifest['release_roster_sha256']}`",
            f"- Topology protocol: `{run_manifest['topology_protocol']}`",
            f"- Fingerprint protocol: `{run_manifest['fingerprint_protocol']}`",
            f"- Unpack method: {run_manifest['unpack_method']}",
            f"- License compliance: {run_manifest['license_compliance']}",
            "",
        ]
    )
    return "\n".join(lines)


def _run_to_output(args: argparse.Namespace, output: Path) -> dict[str, Any]:
    dataset_root = args.dataset_root.resolve()
    output = output.resolve()
    protocol = args.protocol.resolve()
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the SketchMobility dataset root")

    release_manifest_path = dataset_root / "dataset_chunks/manifest.json"
    release_manifest_sha256 = SHARED.sha256_file(release_manifest_path)
    release_manifest, identities = load_release_manifest(dataset_root)
    chunk_verification = verify_chunks(dataset_root, release_manifest)
    roster = build_release_roster(dataset_root, identities)
    SHARED.write_jsonl(output / "release_roster.jsonl", roster)
    release_roster_sha256 = SHARED.sha256_file(output / "release_roster.jsonl")

    complete = [row for row in roster if row["roster_status"] == "COMPLETE"]
    excluded = [row for row in roster if row["roster_status"] != "COMPLETE"]
    if len(complete) != EXPECTED_RELEASE_OBJECT_COUNT and not args.allow_incomplete_release:
        reasons = Counter(row["roster_status"] for row in excluded)
        raise ValueError(
            "release roster is incomplete and --allow-incomplete-release was not set: "
            f"{dict(reasons)}"
        )
    n_release = len(complete)
    release_source_categories = {row["source_category"] for row in complete}
    release_category_names = {row["category"] for row in complete}
    universe_bytes = "".join(
        f"{row['asset_id']}\n" for row in sorted(complete, key=lambda r: r["asset_id"])
    ).encode("utf-8")
    universe_sha256 = hashlib.sha256(universe_bytes).hexdigest()

    selected = freeze_selection(
        complete,
        sample_size=args.sample_size,
        seed=str(args.seed),
        release_manifest_sha256=release_manifest_sha256,
    )
    started_at = datetime.now(timezone.utc).isoformat()
    run_manifest: dict[str, Any] = {
        "schema_version": 1,
        "dataset": "SketchMobility",
        "release_status": "RELEASED_MIXED_LICENSE_SECONDARY_CURATION",
        "paper_reported_assets": 4956,
        "N_release": n_release,
        "N_eval": len(selected),
        "release_excluded_assets": [
            {"asset_id": row["asset_id"], "reason": row["roster_status"], "error": row["roster_error"]}
            for row in excluded
        ],
        "seed": str(args.seed),
        "selection_protocol": SELECTION_PROTOCOL,
        "selection_policy": (
            "SHA256(protocol_id + NUL + release_manifest_sha256 + NUL + seed + NUL + "
            "asset_id), ascending by (digest, asset_id), first N; no replacement or "
            "outcome filtering"
        ),
        "cohort_type": "GLOBAL_FIXED_SAMPLE_NOT_CATEGORY_BALANCED",
        "missing_or_failed_assets_retained": True,
        "category_policy": (
            "declared release pair {source}/{category}, exact strings, no semantic "
            "merging; bare category names are supplementary diagnostics only"
        ),
        "release_manifest": release_manifest_path.relative_to(dataset_root).as_posix(),
        "release_manifest_sha256": release_manifest_sha256,
        "release_manifest_created_at": release_manifest.get("createdAt"),
        "release_manifest_declared_object_count": release_manifest.get("objectCount"),
        "release_manifest_declared_file_count": release_manifest.get("fileCount"),
        "release_manifest_declared_uncompressed_bytes": release_manifest.get(
            "uncompressedBytes"
        ),
        "chunk_verification": chunk_verification,
        "release_universe_sha256": universe_sha256,
        "release_roster": "release_roster.jsonl",
        "release_roster_sha256": release_roster_sha256,
        "release_source_category_count": len(release_source_categories),
        "release_category_name_count": len(release_category_names),
        "protocol": str(protocol),
        "protocol_sha256": SHARED.sha256_file(protocol),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": SHARED.sha256_file(Path(__file__).resolve()),
        "shared_metric_runner": str(Path(SHARED.__file__).resolve()),
        "shared_metric_runner_sha256": SHARED.sha256_file(Path(SHARED.__file__).resolve()),
        "topology_protocol": SHARED.TOPOLOGY_PROTOCOL,
        "fingerprint_protocol": SHARED.FINGERPRINT_PROTOCOL,
        "movable_joint_policy": (
            "all declared XML joint elements whose normalized type is not literal "
            "fixed; includes exporter extension types and does not imply "
            "runtime-valid DoFs"
        ),
        "topology_denominator_policy": (
            "unique topology hashes divided by assets with valid rooted joint trees; "
            "coverage is reported against N_eval"
        ),
        "duplicate_denominator_policy": (
            "duplicate excess divided by assets with complete simulation-package "
            "fingerprints; coverage is reported against N_eval"
        ),
        "p90_definition": "nearest-rank: sorted_values[ceil(0.90 * n) - 1]",
        "unpack_method": UNPACK_METHOD_NOTE,
        "license_compliance": LICENSE_COMPLIANCE_NOTE,
        "workers": args.workers,
        "python_executable": sys.executable,
        "python_version": sys.version,
        "started_at_utc": started_at,
        "assets": [
            {
                key: row[key]
                for key in (
                    "selection_rank",
                    "selection_hash",
                    "asset_id",
                    "source",
                    "category",
                    "source_category",
                    "display_id",
                    "chunk",
                    "declared_file_count",
                    "declared_uncompressed_bytes",
                    "mobility_urdf_sha256",
                    "annotation_sha256",
                )
            }
            for row in selected
        ],
    }
    SHARED.write_json(output / "manifest.json", run_manifest)

    if args.workers == 1:
        records = [_evaluate_asset_fail_closed(dataset_root, row) for row in selected]
    else:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            records = list(
                executor.map(
                    lambda row: _evaluate_asset_fail_closed(dataset_root, row),
                    selected,
                )
            )
    SHARED.write_jsonl(output / "asset_records.jsonl", records)

    summary = aggregate_records(
        records,
        release_asset_count=n_release,
        release_source_category_count=len(release_source_categories),
        release_category_name_count=len(release_category_names),
    )
    summary["status_counts"] = dict(
        sorted(Counter(str(row["status"]) for row in records).items())
    )
    summary["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
    SHARED.write_json(output / "summary.json", summary)
    SHARED._atomic_write_text(output / "report.md", _report(summary, run_manifest))

    artifact_files = (
        "manifest.json",
        "release_roster.jsonl",
        "asset_records.jsonl",
        "summary.json",
        "report.md",
    )
    artifact_manifest = {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "files": {
            name: {
                "bytes": (output / name).stat().st_size,
                "sha256": SHARED.sha256_file(output / name),
            }
            for name in artifact_files
        },
    }
    SHARED.write_json(output / "artifact_manifest.json", artifact_manifest)
    return summary


def run(args: argparse.Namespace) -> dict[str, Any]:
    output = args.output.absolute()
    dataset_root = args.dataset_root.resolve()
    try:
        output.relative_to(dataset_root)
    except ValueError:
        pass
    else:
        raise ValueError("output must not be inside the SketchMobility dataset root")

    with SHARED._output_lock(output):
        staging = output.parent / (
            f".{output.name}.staging.{os.getpid()}.{uuid.uuid4().hex}"
        )
        staging.mkdir(parents=False, exist_ok=False)
        try:
            summary = _run_to_output(args, staging)
            SHARED._verify_staged_artifacts(staging)
            SHARED._publish_staged_output(staging, output)
            return summary
        finally:
            if staging.exists():
                import shutil

                shutil.rmtree(staging)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--sample-size", type=int, default=800)
    parser.add_argument("--seed", default=DEFAULT_SEED)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--allow-incomplete-release",
        action="store_true",
        help=(
            "permit a frozen N_release smaller than the declared release when "
            "individual objects fail runtime roster verification (exclusions are "
            "recorded with reasons)"
        ),
    )
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.sample_size <= 0 or args.workers <= 0:
        raise SystemExit("--sample-size and --workers must be positive")
    summary = run(args)
    print(
        json.dumps(
            {
                "state": "COMPLETE",
                "N_release": summary["cohort"]["N_release"],
                "N_eval": summary["cohort"]["N_eval"],
                "N_parse": summary["cohort"]["N_parse"],
                "output": str(args.output.absolute()),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
