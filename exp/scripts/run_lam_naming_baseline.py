#!/usr/bin/env python3
"""Evaluate the official LAM viable release under Naming protocol v1.1.

The evaluator is deliberately offline: it never executes generated code or
calls a model.  It streams the full official viable tar without extracting it,
audits every archive member, and evaluates only URDF links with at least one
valid renderable visual geometry.  Semantic metrics fail closed unless a
LAM-linked output-independent gold set and three complete blind judges exist.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import posixpath
import random
import re
import statistics
import subprocess
import tarfile
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
SOURCE_ROOT = REPO_ROOT / ".cache" / "table6_sources" / "lam"
CODE_ROOT = SOURCE_ROOT / "code"
DATASET_ROOT = SOURCE_ROOT / "dataset"
MANIFEST_CSV = DATASET_ROOT / "manifest.csv"
MANIFEST_PARQUET = DATASET_ROOT / "manifest.parquet"
CODE_PARQUET = DATASET_ROOT / "articulated_code.parquet"
VIABLE_ARCHIVE = DATASET_ROOT / "viable.tar.gz"
PROTOCOL_PATH = REPO_ROOT / "exp" / "reference" / "baseline_naming_protocol_v1.json"
JUDGE_PROTOCOL_PATH = REPO_ROOT / "exp" / "reference" / "naming_protocol_v2.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp" / "runtime" / "lam_naming_v1"
EXPECTED_ORIGIN = "https://github.com/gaoypeng/LAM.git"
PLACEHOLDER_RE = re.compile(
    r"^(?:link|part|mesh|geometry|object)(?:[_-]?(?:\d+|new|object))?$",
    re.IGNORECASE,
)
MAX_URDF_BYTES = 16 * 1024 * 1024
SEMANTIC_FIELDS = (
    "semantic_precision",
    "semantic_recall",
    "naming_richness",
    "functional_core_coverage",
    "instance_discriminability",
    "over_segmentation_rate",
)


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def require_regular_file(path: Path) -> Path:
    path = contained(path)
    if path.is_symlink() or not path.is_file():
        raise RuntimeError(f"required regular file missing: {path}")
    return path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with require_regular_file(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def write_text_atomic(path: Path, text: str) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def run_git(*args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(contained(CODE_ROOT)), *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def read_manifest() -> tuple[list[dict[str, str]], dict[str, int]]:
    with require_regular_file(MANIFEST_CSV).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {"object_release_id", "category", "tier", "rel_path", "n_links", "n_meshes"}
    if not rows or not required.issubset(rows[0]):
        raise RuntimeError(f"manifest missing columns: {sorted(required - set(rows[0] if rows else {}))}")
    tier_counts = Counter(row["tier"] for row in rows)
    viable = sorted(
        (row for row in rows if row["tier"] == "viable"),
        key=lambda row: (row["rel_path"], row["object_release_id"]),
    )
    release_ids = [row["object_release_id"] for row in viable]
    rel_paths = [row["rel_path"] for row in viable]
    if len(release_ids) != len(set(release_ids)):
        raise RuntimeError("viable object_release_id values are not unique")
    if len(rel_paths) != len(set(rel_paths)):
        raise RuntimeError("viable rel_path values are not unique")
    for rel_path in rel_paths:
        path = PurePosixPath(rel_path)
        if path.is_absolute() or ".." in path.parts or not path.parts or path.parts[0] != "objects":
            raise RuntimeError(f"unsafe or unexpected viable rel_path: {rel_path}")
    return viable, dict(sorted(tier_counts.items()))


def safe_archive_name(name: str) -> tuple[str | None, str | None]:
    if not name or "\\" in name:
        return None, "empty_or_backslash_name"
    path = PurePosixPath(name)
    if path.is_absolute() or ".." in path.parts:
        return None, "absolute_or_parent_traversal"
    normalized = posixpath.normpath(name)
    if normalized in {"", "."} or normalized.startswith("../"):
        return None, "invalid_normalized_name"
    return normalized.removeprefix("./"), None


def stream_archive(expected_urdfs: set[str]) -> tuple[dict[str, int], dict[str, bytes], dict[str, Any]]:
    regular_files: dict[str, int] = {}
    urdfs: dict[str, bytes] = {}
    duplicate_members: list[str] = []
    unsafe_members: list[dict[str, str]] = []
    link_members: list[str] = []
    special_members: list[str] = []
    seen: set[str] = set()
    all_named_generated_urdfs: set[str] = set()
    package_root_generated_urdfs: set[str] = set()
    type_counts: Counter[str] = Counter()
    total_uncompressed_bytes = 0

    with tarfile.open(require_regular_file(VIABLE_ARCHIVE), "r|gz") as archive:
        for member in archive:
            normalized, issue = safe_archive_name(member.name)
            if issue is not None or normalized is None:
                unsafe_members.append({"name": member.name, "reason": issue or "unknown"})
                continue
            if normalized in seen:
                duplicate_members.append(normalized)
            seen.add(normalized)
            if member.isfile():
                type_counts["regular_file"] += 1
                total_uncompressed_bytes += member.size
                regular_files[normalized] = member.size
                if normalized.endswith("/generated.urdf"):
                    all_named_generated_urdfs.add(normalized)
                    parts = PurePosixPath(normalized).parts
                    if len(parts) == 4 and parts[0] == "objects":
                        package_root_generated_urdfs.add(normalized)
                if normalized in expected_urdfs:
                    if member.size > MAX_URDF_BYTES:
                        unsafe_members.append({"name": normalized, "reason": "oversized_urdf"})
                        continue
                    source = archive.extractfile(member)
                    if source is None:
                        unsafe_members.append({"name": normalized, "reason": "unreadable_urdf"})
                        continue
                    urdfs[normalized] = source.read(MAX_URDF_BYTES + 1)
            elif member.isdir():
                type_counts["directory"] += 1
            elif member.issym() or member.islnk():
                type_counts["link"] += 1
                link_members.append(normalized)
            else:
                type_counts["special"] += 1
                special_members.append(normalized)

    missing_expected_urdfs = sorted(expected_urdfs - package_root_generated_urdfs)
    extra_generated_urdfs = sorted(package_root_generated_urdfs - expected_urdfs)
    audit = {
        "member_count": len(seen),
        "type_counts": dict(sorted(type_counts.items())),
        "regular_file_count": len(regular_files),
        "total_uncompressed_bytes": total_uncompressed_bytes,
        "unsafe_member_count": len(unsafe_members),
        "unsafe_members": unsafe_members[:100],
        "duplicate_member_count": len(duplicate_members),
        "duplicate_members": duplicate_members[:100],
        "link_member_count": len(link_members),
        "link_members": link_members[:100],
        "special_member_count": len(special_members),
        "special_members": special_members[:100],
        "streamed_without_extraction": True,
        "expected_generated_urdf_count": len(expected_urdfs),
        "generated_urdf_member_count": len(package_root_generated_urdfs),
        "all_named_generated_urdf_member_count": len(all_named_generated_urdfs),
        "intermediate_log_generated_urdf_count": len(all_named_generated_urdfs - package_root_generated_urdfs),
        "missing_expected_generated_urdf_count": len(missing_expected_urdfs),
        "missing_expected_generated_urdfs": missing_expected_urdfs[:100],
        "extra_generated_urdf_count": len(extra_generated_urdfs),
        "extra_generated_urdfs": extra_generated_urdfs[:100],
        "archive_safe_for_evaluation": not unsafe_members and not duplicate_members,
        "manifest_urdf_set_exact_match": not missing_expected_urdfs and not extra_generated_urdfs,
    }
    return regular_files, urdfs, audit


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def direct_children(node: ET.Element, name: str) -> Iterable[ET.Element]:
    return (child for child in node if local_name(child.tag) == name)


def positive_finite_vector(raw: str | None, count: int) -> bool:
    if raw is None:
        return False
    try:
        values = [float(value) for value in raw.split()]
    except ValueError:
        return False
    return len(values) == count and all(math.isfinite(value) and value > 0 for value in values)


def positive_finite_scalar(raw: str | None) -> bool:
    try:
        value = float(raw)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    return math.isfinite(value) and value > 0


def resolve_mesh_member(package: str, filename: str) -> tuple[str | None, str | None]:
    if not filename or "\\" in filename or "://" in filename:
        return None, "empty_or_nonrelative_mesh_filename"
    path = PurePosixPath(filename)
    if path.is_absolute() or ".." in path.parts:
        return None, "mesh_path_escape"
    normalized = posixpath.normpath(posixpath.join(package, filename))
    package_prefix = package.rstrip("/") + "/"
    if not normalized.startswith(package_prefix):
        return None, "mesh_outside_frozen_package"
    return normalized, None


def geometry_result(
    geometry: ET.Element,
    package: str,
    regular_files: dict[str, int],
) -> tuple[bool, str, str | None]:
    children = list(geometry)
    if len(children) != 1:
        return False, "invalid_geometry", "geometry_child_count_not_one"
    shape = children[0]
    kind = local_name(shape.tag)
    if kind == "mesh":
        member, issue = resolve_mesh_member(package, shape.get("filename", ""))
        if issue is not None or member is None:
            return False, "mesh", issue
        if regular_files.get(member, 0) <= 0:
            return False, "mesh", "mesh_missing_or_empty"
        return True, "mesh", None
    if kind == "box":
        valid = positive_finite_vector(shape.get("size"), 3)
        return valid, "box", None if valid else "invalid_box_size"
    if kind == "cylinder":
        valid = positive_finite_scalar(shape.get("radius")) and positive_finite_scalar(shape.get("length"))
        return valid, "cylinder", None if valid else "invalid_cylinder_dimensions"
    if kind == "sphere":
        valid = positive_finite_scalar(shape.get("radius"))
        return valid, "sphere", None if valid else "invalid_sphere_radius"
    return False, kind or "unknown", "unsupported_geometry_type"


def evaluate_asset(
    manifest_row: dict[str, str],
    urdf_bytes: bytes | None,
    regular_files: dict[str, int],
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "object_release_id": manifest_row["object_release_id"],
        "category": manifest_row["category"],
        "rel_path": manifest_row["rel_path"],
        "artifact_found": urdf_bytes is not None,
        "urdf_sha256": sha256_bytes(urdf_bytes) if urdf_bytes is not None else None,
        "parse_success": False,
        "naming_evaluable": False,
        "link_count": 0,
        "renderable_part_count": 0,
        "named_renderable_part_count": 0,
        "placeholder_renderable_part_count": 0,
        "nameability": None,
        "valid_visual_geometry_count": 0,
        "invalid_visual_geometry_count": 0,
        "valid_geometry_types": {},
        "invalid_geometry_reasons": {},
        "placeholder_names": {},
        "manifest_n_links": int(manifest_row["n_links"] or 0),
        "manifest_n_meshes": int(manifest_row["n_meshes"] or 0),
        "issues": [],
    }
    if urdf_bytes is None:
        record["issues"].append("generated.urdf missing from viable archive")
        return record
    try:
        root = ET.fromstring(urdf_bytes)
    except (ET.ParseError, ValueError) as exc:
        record["issues"].append(f"urdf_parse: {type(exc).__name__}: {exc}")
        return record
    record["parse_success"] = True
    links = [node for node in root.iter() if local_name(node.tag) == "link"]
    record["link_count"] = len(links)
    geometry_types: Counter[str] = Counter()
    invalid_reasons: Counter[str] = Counter()
    placeholder_names: Counter[str] = Counter()
    part_count = named_count = valid_geometry_count = invalid_geometry_count = 0
    for link in links:
        link_has_renderable_geometry = False
        for visual in direct_children(link, "visual"):
            geometry_nodes = list(direct_children(visual, "geometry"))
            if not geometry_nodes:
                invalid_geometry_count += 1
                invalid_reasons["visual_without_geometry"] += 1
                continue
            for geometry in geometry_nodes:
                valid, kind, issue = geometry_result(geometry, manifest_row["rel_path"], regular_files)
                if valid:
                    valid_geometry_count += 1
                    geometry_types[kind] += 1
                    link_has_renderable_geometry = True
                else:
                    invalid_geometry_count += 1
                    invalid_reasons[issue or "invalid_geometry"] += 1
        if not link_has_renderable_geometry:
            continue
        part_count += 1
        name = link.get("name", "")
        if name and PLACEHOLDER_RE.fullmatch(name) is None:
            named_count += 1
        else:
            placeholder_names[name or "<empty>"] += 1
    record.update(
        {
            "naming_evaluable": part_count > 0,
            "renderable_part_count": part_count,
            "named_renderable_part_count": named_count,
            "placeholder_renderable_part_count": part_count - named_count,
            "nameability": named_count / part_count if part_count else None,
            "valid_visual_geometry_count": valid_geometry_count,
            "invalid_visual_geometry_count": invalid_geometry_count,
            "valid_geometry_types": dict(sorted(geometry_types.items())),
            "invalid_geometry_reasons": dict(sorted(invalid_reasons.items())),
            "placeholder_names": dict(sorted(placeholder_names.items())),
        }
    )
    if len(links) != record["manifest_n_links"]:
        record["issues"].append("parsed link count differs from manifest n_links")
    return record


def bootstrap_ci(records: list[dict[str, Any]], resamples: int, confidence: float, seed: int) -> dict[str, Any]:
    if not records:
        return {"parts_mean": None, "nameability_micro": None}
    parts = [row["renderable_part_count"] for row in records]
    named = [row["named_renderable_part_count"] for row in records]
    rng = random.Random(seed)
    part_means: list[float] = []
    nameability: list[float] = []
    count = len(records)
    for _ in range(resamples):
        sampled = [rng.randrange(count) for _ in range(count)]
        sampled_parts = sum(parts[index] for index in sampled)
        sampled_named = sum(named[index] for index in sampled)
        part_means.append(sampled_parts / count)
        nameability.append(sampled_named / sampled_parts if sampled_parts else math.nan)
    alpha = (1.0 - confidence) / 2.0

    def percentile(values: list[float], probability: float) -> float:
        ordered = sorted(value for value in values if math.isfinite(value))
        position = probability * (len(ordered) - 1)
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return ordered[lower]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    return {
        "method": "asset bootstrap with replacement",
        "resamples": resamples,
        "confidence": confidence,
        "seed": seed,
        "parts_mean": [percentile(part_means, alpha), percentile(part_means, 1.0 - alpha)],
        "nameability_micro": [percentile(nameability, alpha), percentile(nameability, 1.0 - alpha)],
    }


def aggregate(
    manifest_rows: list[dict[str, str]],
    records: list[dict[str, Any]],
    tier_counts: dict[str, int],
    archive_audit: dict[str, Any],
    protocol: dict[str, Any],
) -> dict[str, Any]:
    evaluable = [row for row in records if row["naming_evaluable"]]
    total_parts = sum(row["renderable_part_count"] for row in evaluable)
    total_named = sum(row["named_renderable_part_count"] for row in evaluable)
    placeholder_counts: Counter[str] = Counter()
    geometry_types: Counter[str] = Counter()
    invalid_reasons: Counter[str] = Counter()
    for row in records:
        placeholder_counts.update(row["placeholder_names"])
        geometry_types.update(row["valid_geometry_types"])
        invalid_reasons.update(row["invalid_geometry_reasons"])
    manifest_link_mismatch = sum(row["link_count"] != row["manifest_n_links"] for row in records if row["parse_success"])
    status = "COMPLETE"
    if not archive_audit["archive_safe_for_evaluation"]:
        status = "BLOCKED_ARCHIVE_AMBIGUITY"
    elif not archive_audit["manifest_urdf_set_exact_match"] or len(evaluable) != len(manifest_rows):
        status = "COMPLETE_WITH_ARTIFACT_FAILURES"
    bootstrap = protocol["bootstrap"]
    direct_ci = bootstrap_ci(
        evaluable,
        int(bootstrap["resamples"]),
        float(bootstrap["confidence"]),
        int(bootstrap["seed"]),
    )
    records_payload = "".join(
        json.dumps(row, sort_keys=True, ensure_ascii=False, separators=(",", ":")) + "\n"
        for row in records
    )
    stable_metrics = {
        "requested_assets": len(manifest_rows),
        "artifact_found_assets": sum(row["artifact_found"] for row in records),
        "parse_success_assets": sum(row["parse_success"] for row in records),
        "naming_evaluable_assets": len(evaluable),
        "total_renderable_parts": total_parts,
        "total_named_renderable_parts": total_named,
        "parts_per_asset_mean": statistics.fmean(row["renderable_part_count"] for row in evaluable) if evaluable else None,
        "parts_per_asset_median": statistics.median(row["renderable_part_count"] for row in evaluable) if evaluable else None,
        "nameability_micro": total_named / total_parts if total_parts else None,
        "nameability_asset_macro": statistics.fmean(row["nameability"] for row in evaluable) if evaluable else None,
        "assets_with_placeholder_parts": sum(row["placeholder_renderable_part_count"] > 0 for row in evaluable),
        "fully_nameable_assets": sum(row["placeholder_renderable_part_count"] == 0 for row in evaluable),
    }
    return {
        "protocol_id": "nano3d_lam_naming_release_v1.1",
        "common_protocol_id": protocol["protocol_id"],
        "status": status,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "network_accessed": False,
        "generated_code_executed": False,
        "paper_values_reused": False,
        "comparison_scope": "official viable release cohort; not a shared prompt-matched rerun against other methods",
        "release": {
            "code_repository": run_git("remote", "get-url", "origin"),
            "code_commit": run_git("rev-parse", "HEAD"),
            "code_worktree_clean": run_git("status", "--short", "--untracked-files=no") == "",
            "official_origin_match": run_git("remote", "get-url", "origin") == EXPECTED_ORIGIN,
            "dataset": "YipengGao/Articulated-Object-Code",
            "cohort": "all manifest rows with tier=viable",
        },
        "coverage": {
            "release_manifest_rows": sum(tier_counts.values()),
            "tier_counts": tier_counts,
            "requested_viable_assets": len(manifest_rows),
            "requested_unique_release_ids": len({row["object_release_id"] for row in manifest_rows}),
            "requested_unique_rel_paths": len({row["rel_path"] for row in manifest_rows}),
            "requested_categories": len({row["category"] for row in manifest_rows}),
            "artifact_found_assets": stable_metrics["artifact_found_assets"],
            "parse_success_assets": stable_metrics["parse_success_assets"],
            "naming_evaluable_assets": stable_metrics["naming_evaluable_assets"],
            "failed_artifact_assets": len(manifest_rows) - len(evaluable),
            "manifest_link_count_mismatch_assets": manifest_link_mismatch,
        },
        "archive_audit": archive_audit,
        "direct_metrics": {
            **stable_metrics,
            "parts_per_asset_min": min((row["renderable_part_count"] for row in evaluable), default=None),
            "parts_per_asset_max": max((row["renderable_part_count"] for row in evaluable), default=None),
            "placeholder_renderable_parts": total_parts - total_named,
            "placeholder_name_counts": dict(placeholder_counts.most_common()),
            "valid_visual_geometry_count": sum(row["valid_visual_geometry_count"] for row in records),
            "invalid_visual_geometry_count": sum(row["invalid_visual_geometry_count"] for row in records),
            "valid_geometry_types": dict(sorted(geometry_types.items())),
            "invalid_geometry_reasons": dict(sorted(invalid_reasons.items())),
            "bootstrap_ci": direct_ci,
            "representation": "URDF renderable-link; not merged with GLB-node results",
        },
        "cross_seed": {
            "eligible": False,
            "status": "N/A",
            "reason": "LAM release rows are independent per-asset generations and expose no frozen reusable factory/template plus seed identity",
            "raw_unique_name_set_jaccard": None,
            "raw_name_multiset_weighted_jaccard": None,
            "exact_raw_name_multiset_mode_rate": None,
        },
        "semantic_evidence": {
            "output_independent_lam_role_gold": False,
            "lam_linked_complete_blind_judges": 0,
            "required_blind_judges": 3,
            "existing_naming_gold_is_lam_linked": False,
            "existing_judge_packet_is_lam_linked": False,
        },
        "semantic_metrics": {field: None for field in SEMANTIC_FIELDS},
        "semantic_status": "N/A: no LAM-linked output-independent role gold and no three complete independent blind judges",
        "provenance": {
            "manifest_csv_sha256": sha256_file(MANIFEST_CSV),
            "manifest_parquet_sha256": sha256_file(MANIFEST_PARQUET),
            "articulated_code_parquet_sha256": sha256_file(CODE_PARQUET),
            "viable_archive_sha256": sha256_file(VIABLE_ARCHIVE),
            "dataset_readme_sha256": sha256_file(DATASET_ROOT / "README.md"),
            "code_agents_sha256": sha256_file(CODE_ROOT / "AGENTS.md"),
            "common_protocol_sha256": sha256_file(PROTOCOL_PATH),
            "judge_protocol_sha256": sha256_file(JUDGE_PROTOCOL_PATH),
            "evaluator_sha256": sha256_file(Path(__file__)),
            "records_sha256": sha256_bytes(records_payload.encode()),
            "stable_metrics_sha256": sha256_bytes(json.dumps(stable_metrics, sort_keys=True, separators=(",", ":")).encode()),
        },
    }


def render_report(summary: dict[str, Any]) -> str:
    coverage = summary["coverage"]
    direct = summary["direct_metrics"]
    archive = summary["archive_audit"]
    return f"""# LAM Naming baseline: official viable release

Status: **{summary['status']}**

Protocol: `{summary['common_protocol_id']}`. This is an offline local evaluation
of the official LAM viable release, not a transcription of paper values.

## Coverage

- Frozen requested cohort: {coverage['requested_viable_assets']} viable assets across {coverage['requested_categories']} categories.
- URDF artifacts found: {coverage['artifact_found_assets']}/{coverage['requested_viable_assets']}.
- URDF parse success: {coverage['parse_success_assets']}/{coverage['requested_viable_assets']}.
- Naming-evaluable artifacts: {coverage['naming_evaluable_assets']}/{coverage['requested_viable_assets']}.
- Manifest/parsed link-count mismatches: {coverage['manifest_link_count_mismatch_assets']}.

## Direct Naming results

- Parts: {direct['total_renderable_parts']} renderable URDF links; {direct['parts_per_asset_mean']:.6f} mean per evaluable asset (median {direct['parts_per_asset_median']}).
- Named / Nameability: {direct['total_named_renderable_parts']}/{direct['total_renderable_parts']} = {direct['nameability_micro']:.6f} micro; asset-macro {direct['nameability_asset_macro']:.6f}.
- Placeholder renderable parts: {direct['placeholder_renderable_parts']}.
- Assets containing placeholder parts: {direct['assets_with_placeholder_parts']}/{direct['naming_evaluable_assets']}; fully nameable assets: {direct['fully_nameable_assets']}/{direct['naming_evaluable_assets']}.
- Representation: URDF renderable-link only; do not merge this row with GLB-node counts.

This is an official-release cohort audit, not a shared prompt/category-matched
rerun against the other methods; it should be labeled separately in Table 2.

The archive was streamed without extraction. It contains {archive['member_count']}
unique members and {archive['regular_file_count']} regular files; unsafe names={archive['unsafe_member_count']},
duplicates={archive['duplicate_member_count']}, links={archive['link_member_count']}, special members={archive['special_member_count']}.
The {archive['intermediate_log_generated_urdf_count']} additional `pipeline_logs/.../generated.urdf`
members are intermediate feedback iterations; they are recorded but excluded
from the canonical final-artifact cohort and were not evaluated.

## Fail-closed metrics

Cross-seed consistency is `N/A`: LAM is evaluated as independent per-asset
generation and the release has no reusable factory/template plus seed identity.
Semantic Precision/Recall, Naming Richness, Functional Core Coverage, Instance
Discriminability, and Over-Segmentation remain `N/A`: no LAM-linked
output-independent role gold or three complete independent blind judges exist.

## Provenance

- Official code: `{summary['release']['code_repository']}` at `{summary['release']['code_commit']}`.
- Manifest SHA256: `{summary['provenance']['manifest_csv_sha256']}`.
- Viable tar SHA256: `{summary['provenance']['viable_archive_sha256']}`.
- Evaluator SHA256: `{summary['provenance']['evaluator_sha256']}`.
- Records SHA256: `{summary['provenance']['records_sha256']}`.
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output_dir = contained(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    protocol = json.loads(require_regular_file(PROTOCOL_PATH).read_text(encoding="utf-8"))
    if protocol.get("protocol_id") != "nano3d_table2_baseline_naming_v1.1":
        raise RuntimeError(f"unexpected common protocol: {protocol.get('protocol_id')}")
    manifest_rows, tier_counts = read_manifest()
    expected_urdfs = {f"{row['rel_path'].rstrip('/')}/generated.urdf" for row in manifest_rows}
    regular_files, urdfs, archive_audit = stream_archive(expected_urdfs)
    records = [
        evaluate_asset(row, urdfs.get(f"{row['rel_path'].rstrip('/')}/generated.urdf"), regular_files)
        for row in manifest_rows
    ]
    summary = aggregate(manifest_rows, records, tier_counts, archive_audit, protocol)
    records_payload = "".join(
        json.dumps(record, sort_keys=True, ensure_ascii=False, separators=(",", ":")) + "\n"
        for record in records
    )
    write_text_atomic(output_dir / "records.jsonl", records_payload)
    write_text_atomic(output_dir / "summary.json", json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    write_text_atomic(output_dir / "report.md", render_report(summary))
    print(
        json.dumps(
            {
                "status": summary["status"],
                "requested_assets": summary["coverage"]["requested_viable_assets"],
                "evaluable_assets": summary["coverage"]["naming_evaluable_assets"],
                "parts": summary["direct_metrics"]["total_renderable_parts"],
                "nameability": summary["direct_metrics"]["nameability_micro"],
                "output_dir": str(output_dir),
            }
        )
    )
    return 0 if summary["status"] in {"COMPLETE", "COMPLETE_WITH_ARTIFACT_FAILURES"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
