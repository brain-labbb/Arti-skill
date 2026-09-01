#!/usr/bin/env python3
"""Run the frozen Articraft direct Naming baseline without model generation.

The harness selects one retained, hydrated dataset record per Articraft category,
compiles the existing source program to a native URDF package, and evaluates only
mesh-bearing URDF links.  Source names are never used as semantic gold.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import re
import shutil
import statistics
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
RECORDS_ROOT = ARTICRAFT_ROOT / "data" / "records"
DEFAULT_PROTOCOL = REPO_ROOT / "exp" / "reference" / "baseline_naming_protocol_v1.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp" / "runtime" / "articraft_naming_v1"

if str(ARTICRAFT_ROOT) not in sys.path:
    sys.path.insert(0, str(ARTICRAFT_ROOT))

from agent.compiler import compile_urdf_report_maybe_timeout  # noqa: E402


TERMINAL_STATUSES = {"PASS", "COMPILE_ERROR", "TIMEOUT", "PACKAGE_ERROR"}
LFS_POINTER_RE = re.compile(
    rb"^version https://git-lfs.github.com/spec/v1\n"
    rb"oid sha256:([0-9a-f]{64})\nsize ([0-9]+)\n?$"
)


@dataclass(frozen=True)
class Candidate:
    record_id: str
    category_slug: str
    rating: int
    model_path: Path
    provenance_path: Path
    prompt_path: Path | None
    inputs_path: Path | None
    record_path: Path
    dataset_path: Path
    model_sha256: str
    provenance_sha256: str
    prompt_sha256: str | None
    record_sha256: str
    provider: str | None
    model_id: str | None
    active_revision_id: str
    selection_digest: str


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def canonical_inside(path: Path, root: Path, *, must_exist: bool = True) -> Path:
    resolved_root = root.resolve(strict=True)
    resolved = path.resolve(strict=must_exist)
    if resolved != resolved_root and resolved_root not in resolved.parents:
        raise ValueError(f"Path escapes authorized root: {path} -> {resolved}")
    return resolved


def relative_artifact_path(record_dir: Path, value: object) -> Path | None:
    if not isinstance(value, str) or not value.strip():
        return None
    raw = Path(value)
    if raw.is_absolute():
        return None
    candidate = record_dir / raw
    try:
        return canonical_inside(candidate, WORKSPACE_ROOT)
    except (FileNotFoundError, RuntimeError, ValueError):
        return None


def read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return payload


def dump_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def git_commit(path: Path) -> str | None:
    """Resolve a local Git HEAD without consulting system/global Git config."""
    git_marker = canonical_inside(path / ".git", REPO_ROOT)
    if git_marker.is_file():
        marker = git_marker.read_text(encoding="utf-8").strip()
        if not marker.startswith("gitdir: "):
            return None
        raw_git_dir = Path(marker.removeprefix("gitdir: ").strip())
        git_dir = canonical_inside(git_marker.parent / raw_git_dir, REPO_ROOT)
    else:
        git_dir = git_marker
    head = (git_dir / "HEAD").read_text(encoding="utf-8").strip()
    if re.fullmatch(r"[0-9a-f]{40}", head):
        return head
    if not head.startswith("ref: "):
        return None
    ref_name = head.removeprefix("ref: ").strip()
    ref_path = git_dir / ref_name
    if ref_path.is_file():
        value = canonical_inside(ref_path, REPO_ROOT).read_text(encoding="utf-8").strip()
        return value if re.fullmatch(r"[0-9a-f]{40}", value) else None
    packed_refs = git_dir / "packed-refs"
    if not packed_refs.is_file():
        return None
    for line in canonical_inside(packed_refs, REPO_ROOT).read_text(encoding="utf-8").splitlines():
        if not line or line.startswith(("#", "^")):
            continue
        value, _, name = line.partition(" ")
        if name == ref_name and re.fullmatch(r"[0-9a-f]{40}", value):
            return value
    return None


def audit_candidates(
    protocol_id: str,
    records_root: Path = RECORDS_ROOT,
) -> tuple[list[Candidate], dict[str, Any]]:
    counts: Counter[str] = Counter()
    rating_counts: Counter[str] = Counter()
    candidates: list[Candidate] = []

    records_root = canonical_inside(records_root, WORKSPACE_ROOT)
    for entry in sorted(records_root.iterdir(), key=lambda item: item.name):
        if entry.is_symlink() or not entry.is_dir():
            continue
        counts["record_directories"] += 1
        record_path = entry / "record.json"
        try:
            canonical_inside(record_path, WORKSPACE_ROOT)
            record_bytes = record_path.read_bytes()
            record = json.loads(record_bytes)
            if not isinstance(record, dict):
                raise ValueError("record payload is not an object")
        except (FileNotFoundError, json.JSONDecodeError, OSError, RuntimeError, ValueError):
            counts["unhydrated_or_invalid_record_payload"] += 1
            continue

        counts["parseable_records"] += 1
        rating = record.get("rating")
        rating_counts[str(rating)] += 1
        if "dataset" not in record.get("collections", []):
            counts["not_dataset_collection"] += 1
            continue
        counts["dataset_records"] += 1
        if rating not in (4, 5):
            counts["dataset_not_primary_rating_4_or_5"] += 1
            continue
        counts["retained_dataset_records"] += 1

        record_id = record.get("record_id")
        category_slug = record.get("category_slug")
        active_revision_id = record.get("active_revision_id")
        if record_id != entry.name or not isinstance(category_slug, str) or not category_slug:
            counts["invalid_record_identity_or_category"] += 1
            continue
        if not isinstance(active_revision_id, str) or not active_revision_id:
            counts["missing_active_revision"] += 1
            continue

        artifacts = record.get("artifacts")
        if not isinstance(artifacts, dict):
            counts["missing_artifact_map"] += 1
            continue
        model_path = relative_artifact_path(entry, artifacts.get("model_py"))
        provenance_path = relative_artifact_path(entry, artifacts.get("provenance_json"))
        prompt_path = relative_artifact_path(entry, artifacts.get("prompt_txt"))
        inputs_path = relative_artifact_path(entry, artifacts.get("inputs_dir"))
        if model_path is None or provenance_path is None:
            counts["missing_model_or_provenance"] += 1
            continue
        if not model_path.is_file() or not provenance_path.is_file():
            counts["missing_model_or_provenance"] += 1
            continue

        dataset_path = entry / "collections" / "dataset.json"
        try:
            dataset = read_json(canonical_inside(dataset_path, WORKSPACE_ROOT))
        except (FileNotFoundError, json.JSONDecodeError, OSError, RuntimeError, ValueError):
            counts["missing_or_invalid_dataset_sidecar"] += 1
            continue
        if dataset.get("record_id") != record_id or dataset.get("category_slug") != category_slug:
            counts["dataset_sidecar_mismatch"] += 1
            continue

        model_hash = sha256_file(model_path)
        declared_hashes = record.get("hashes")
        declared_model_hash = (
            declared_hashes.get("model_py_sha256") if isinstance(declared_hashes, dict) else None
        )
        if declared_model_hash != model_hash:
            counts["model_hash_mismatch"] += 1
            continue

        provenance_hash = sha256_file(provenance_path)
        prompt_hash = sha256_file(prompt_path) if prompt_path and prompt_path.is_file() else None
        selection_payload = "\0".join(
            (protocol_id, category_slug, record_id, model_hash)
        ).encode("utf-8")
        candidates.append(
            Candidate(
                record_id=record_id,
                category_slug=category_slug,
                rating=int(rating),
                model_path=model_path,
                provenance_path=provenance_path,
                prompt_path=prompt_path if prompt_path and prompt_path.is_file() else None,
                inputs_path=inputs_path if inputs_path and inputs_path.is_dir() else None,
                record_path=record_path,
                dataset_path=dataset_path,
                model_sha256=model_hash,
                provenance_sha256=provenance_hash,
                prompt_sha256=prompt_hash,
                record_sha256=sha256_bytes(record_bytes),
                provider=str(record.get("provider")) if record.get("provider") else None,
                model_id=str(record.get("model_id")) if record.get("model_id") else None,
                active_revision_id=active_revision_id,
                selection_digest=sha256_bytes(selection_payload),
            )
        )
        counts["eligible_hydrated_retained_records"] += 1

    category_counts = Counter(candidate.category_slug for candidate in candidates)
    return candidates, {
        "counts": dict(sorted(counts.items())),
        "parseable_rating_counts": dict(sorted(rating_counts.items())),
        "eligible_category_count": len(category_counts),
        "eligible_records_per_category": dict(sorted(category_counts.items())),
    }


def safe_record_relative_path(record_id: str, value: object) -> Path | None:
    if not isinstance(value, str) or not value.strip():
        return None
    raw = Path(value)
    if raw.is_absolute() or ".." in raw.parts:
        return None
    return Path(record_id) / raw


def resolve_lfs_overlay_file(
    checkout_records_root: Path,
    mirror_records_root: Path,
    relative_path: Path,
) -> tuple[Path | None, dict[str, Any]]:
    if relative_path.is_absolute() or ".." in relative_path.parts:
        return None, {"status": "unsafe_relative_path", "verified": False}
    checkout_path = checkout_records_root / relative_path
    try:
        checkout_path = canonical_inside(checkout_path, WORKSPACE_ROOT)
    except (FileNotFoundError, RuntimeError, ValueError):
        return None, {"status": "checkout_missing", "verified": False}
    checkout_bytes = checkout_path.read_bytes()
    pointer = LFS_POINTER_RE.fullmatch(checkout_bytes)
    if pointer is None:
        return checkout_path, {
            "status": "checkout_hydrated",
            "verified": True,
            "checkout_path": checkout_path.relative_to(WORKSPACE_ROOT).as_posix(),
            "resolved_path": checkout_path.relative_to(WORKSPACE_ROOT).as_posix(),
            "payload_sha256": sha256_bytes(checkout_bytes),
            "payload_size": len(checkout_bytes),
            "lfs_oid": None,
        }

    mirror_path = mirror_records_root / relative_path
    try:
        mirror_path = canonical_inside(mirror_path, WORKSPACE_ROOT)
    except (FileNotFoundError, RuntimeError, ValueError):
        return None, {
            "status": "lfs_payload_missing",
            "verified": False,
            "checkout_path": checkout_path.relative_to(WORKSPACE_ROOT).as_posix(),
            "lfs_oid": pointer.group(1).decode("ascii"),
        }
    mirror_bytes = mirror_path.read_bytes()
    expected_hash = pointer.group(1).decode("ascii")
    expected_size = int(pointer.group(2))
    actual_hash = sha256_bytes(mirror_bytes)
    if actual_hash != expected_hash or len(mirror_bytes) != expected_size:
        return None, {
            "status": "lfs_payload_mismatch",
            "verified": False,
            "checkout_path": checkout_path.relative_to(WORKSPACE_ROOT).as_posix(),
            "resolved_path": mirror_path.relative_to(WORKSPACE_ROOT).as_posix(),
            "lfs_oid": expected_hash,
            "expected_size": expected_size,
            "actual_sha256": actual_hash,
            "actual_size": len(mirror_bytes),
        }
    return mirror_path, {
        "status": "lfs_oid_mirror",
        "verified": True,
        "checkout_path": checkout_path.relative_to(WORKSPACE_ROOT).as_posix(),
        "resolved_path": mirror_path.relative_to(WORKSPACE_ROOT).as_posix(),
        "payload_sha256": actual_hash,
        "payload_size": len(mirror_bytes),
        "lfs_oid": expected_hash,
    }


def audit_overlay_candidates(
    protocol_id: str,
    checkout_records_root: Path,
    mirror_records_root: Path,
) -> tuple[list[Candidate], dict[str, Any], dict[str, dict[str, Any]]]:
    checkout_records_root = canonical_inside(checkout_records_root, WORKSPACE_ROOT)
    mirror_records_root = canonical_inside(mirror_records_root, WORKSPACE_ROOT)
    counts: Counter[str] = Counter()
    rating_counts: Counter[str] = Counter()
    resolution_counts: Counter[str] = Counter()
    candidates: list[Candidate] = []
    provenance_by_id: dict[str, dict[str, Any]] = {}

    for entry in sorted(checkout_records_root.iterdir(), key=lambda item: item.name):
        if entry.is_symlink() or not entry.is_dir():
            continue
        record_id = entry.name
        counts["record_directories"] += 1
        record_path, record_evidence = resolve_lfs_overlay_file(
            checkout_records_root,
            mirror_records_root,
            Path(record_id) / "record.json",
        )
        resolution_counts[str(record_evidence.get("status"))] += 1
        if record_path is None:
            counts["unresolved_record_payload"] += 1
            continue
        try:
            record_bytes = record_path.read_bytes()
            record = json.loads(record_bytes)
            if not isinstance(record, dict):
                raise ValueError("record payload is not an object")
        except (json.JSONDecodeError, OSError, ValueError):
            counts["invalid_record_payload"] += 1
            continue
        counts["parseable_records"] += 1
        rating = record.get("rating")
        rating_counts[str(rating)] += 1
        if "dataset" not in record.get("collections", []):
            counts["not_dataset_collection"] += 1
            continue
        counts["dataset_records"] += 1
        if rating not in (4, 5):
            counts["dataset_not_primary_rating_4_or_5"] += 1
            continue
        counts["retained_dataset_records"] += 1

        category_slug = record.get("category_slug")
        active_revision_id = record.get("active_revision_id")
        if record.get("record_id") != record_id or not isinstance(category_slug, str):
            counts["invalid_record_identity_or_category"] += 1
            continue
        if not category_slug or not isinstance(active_revision_id, str) or not active_revision_id:
            counts["invalid_record_identity_or_category"] += 1
            continue
        artifacts = record.get("artifacts")
        if not isinstance(artifacts, dict):
            counts["missing_artifact_map"] += 1
            continue

        relative_paths = {
            "record_json": Path(record_id) / "record.json",
            "dataset_json": Path(record_id) / "collections" / "dataset.json",
            "model_py": safe_record_relative_path(record_id, artifacts.get("model_py")),
            "provenance_json": safe_record_relative_path(
                record_id, artifacts.get("provenance_json")
            ),
            "prompt_txt": safe_record_relative_path(record_id, artifacts.get("prompt_txt")),
        }
        if any(value is None for value in relative_paths.values()):
            counts["unsafe_or_missing_required_artifact_path"] += 1
            continue
        resolved: dict[str, Path] = {"record_json": record_path}
        evidence: dict[str, Any] = {"record_json": record_evidence}
        failed = False
        for key in ("dataset_json", "model_py", "provenance_json", "prompt_txt"):
            path, item_evidence = resolve_lfs_overlay_file(
                checkout_records_root,
                mirror_records_root,
                relative_paths[key],  # type: ignore[arg-type]
            )
            resolution_counts[str(item_evidence.get("status"))] += 1
            evidence[key] = item_evidence
            if path is None:
                failed = True
            else:
                resolved[key] = path
        if failed:
            counts["unresolved_required_artifact"] += 1
            continue

        try:
            dataset = read_json(resolved["dataset_json"])
        except (json.JSONDecodeError, OSError, ValueError):
            counts["invalid_dataset_sidecar"] += 1
            continue
        if dataset.get("record_id") != record_id or dataset.get("category_slug") != category_slug:
            counts["dataset_sidecar_mismatch"] += 1
            continue
        model_hash = sha256_file(resolved["model_py"])
        declared_hashes = record.get("hashes")
        declared_model_hash = (
            declared_hashes.get("model_py_sha256")
            if isinstance(declared_hashes, dict)
            else None
        )
        if declared_model_hash != model_hash:
            counts["model_hash_mismatch"] += 1
            continue

        inputs_relative = safe_record_relative_path(record_id, artifacts.get("inputs_dir"))
        checkout_inputs = (
            checkout_records_root / inputs_relative if inputs_relative is not None else None
        )
        mirror_inputs = (
            mirror_records_root / inputs_relative if inputs_relative is not None else None
        )
        inputs_path = None
        if checkout_inputs is not None and checkout_inputs.is_dir():
            inputs_path = canonical_inside(checkout_inputs, WORKSPACE_ROOT)
        elif mirror_inputs is not None and mirror_inputs.is_dir():
            inputs_path = canonical_inside(mirror_inputs, WORKSPACE_ROOT)

        provenance_by_id[record_id] = {
            "all_required_files_verified": all(
                bool(item.get("verified")) for item in evidence.values()
            ),
            "files": evidence,
        }
        prompt_hash = sha256_file(resolved["prompt_txt"])
        selection_payload = "\0".join(
            (protocol_id, category_slug, record_id, model_hash)
        ).encode("utf-8")
        candidates.append(
            Candidate(
                record_id=record_id,
                category_slug=category_slug,
                rating=int(rating),
                model_path=resolved["model_py"],
                provenance_path=resolved["provenance_json"],
                prompt_path=resolved["prompt_txt"],
                inputs_path=inputs_path,
                record_path=record_path,
                dataset_path=resolved["dataset_json"],
                model_sha256=model_hash,
                provenance_sha256=sha256_file(resolved["provenance_json"]),
                prompt_sha256=prompt_hash,
                record_sha256=sha256_bytes(record_bytes),
                provider=str(record.get("provider")) if record.get("provider") else None,
                model_id=str(record.get("model_id")) if record.get("model_id") else None,
                active_revision_id=active_revision_id,
                selection_digest=sha256_bytes(selection_payload),
            )
        )
        counts["eligible_hydrated_retained_records"] += 1

    category_counts = Counter(candidate.category_slug for candidate in candidates)
    return candidates, {
        "mode": "current_checkout_with_verified_lfs_overlay",
        "checkout_records_root": checkout_records_root.relative_to(WORKSPACE_ROOT).as_posix(),
        "mirror_records_root": mirror_records_root.relative_to(WORKSPACE_ROOT).as_posix(),
        "counts": dict(sorted(counts.items())),
        "parseable_rating_counts": dict(sorted(rating_counts.items())),
        "resolution_counts": dict(sorted(resolution_counts.items())),
        "eligible_category_count": len(category_counts),
        "eligible_records_per_category": dict(sorted(category_counts.items())),
    }, provenance_by_id


def select_cohort(candidates: list[Candidate], max_categories: int) -> list[Candidate]:
    by_category: dict[str, list[Candidate]] = {}
    for candidate in candidates:
        by_category.setdefault(candidate.category_slug, []).append(candidate)
    selected = [
        min(items, key=lambda item: (item.selection_digest, item.record_id))
        for _, items in sorted(by_category.items())
    ]
    selected.sort(key=lambda item: item.category_slug)
    if max_categories > 0:
        selected = selected[:max_categories]
    return selected


def select_existing_cohort(
    candidates: list[Candidate],
    prior_manifest: dict[str, Any],
    prior_manifest_sha256: str,
) -> tuple[list[Candidate], dict[str, Any], dict[str, str]]:
    by_id = {candidate.record_id: candidate for candidate in candidates}
    raw_records = prior_manifest.get("records")
    if not isinstance(raw_records, list) or not raw_records:
        raise ValueError("Prior cohort manifest has no records")
    selected: list[Candidate] = []
    selection_digests: dict[str, str] = {}
    for item in raw_records:
        if not isinstance(item, dict):
            raise ValueError("Prior cohort manifest contains a non-object record")
        record_id = item.get("record_id")
        if not isinstance(record_id, str) or record_id not in by_id:
            raise ValueError(f"Prior cohort record is no longer eligible: {record_id}")
        selected.append(by_id[record_id])
        digest = item.get("selection_digest")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError(f"Prior selection digest is invalid: {record_id}")
        selection_digests[record_id] = digest
    selected.sort(key=lambda item: item.category_slug)
    if len({item.category_slug for item in selected}) != len(selected):
        raise ValueError("Prior cohort is not one-record-per-category")
    prior_selection = prior_manifest.get("selection")
    if not isinstance(prior_selection, dict):
        prior_selection = {}
    return selected, {
        "description": "reuse the exact 123 record IDs frozen by the initial v1 cohort; only the evaluator protocol changes",
        "selection_protocol_id": prior_selection.get(
            "selection_protocol_id", prior_manifest.get("protocol_id")
        ),
        "selection_protocol_sha256": prior_selection.get(
            "selection_protocol_sha256", prior_manifest.get("protocol_sha256")
        ),
        "prior_manifest_sha256": prior_selection.get(
            "prior_manifest_sha256", prior_manifest_sha256
        ),
        "retention_rule": prior_selection.get("retention_rule"),
        "hydration_rule": prior_selection.get("hydration_rule"),
        "record_count": len(selected),
    }, selection_digests


def select_expanded_cohort(
    candidates: list[Candidate],
    prior_manifest: dict[str, Any],
    prior_manifest_sha256: str,
) -> tuple[list[Candidate], dict[str, Any], dict[str, str]]:
    by_id = {candidate.record_id: candidate for candidate in candidates}
    by_category: dict[str, list[Candidate]] = {}
    for candidate in candidates:
        by_category.setdefault(candidate.category_slug, []).append(candidate)
    for items in by_category.values():
        items.sort(key=lambda item: (item.selection_digest, item.record_id))

    raw_records = prior_manifest.get("records")
    if not isinstance(raw_records, list) or not raw_records:
        raise ValueError("Prior cohort manifest has no records")
    selected: list[Candidate] = []
    selection_digests: dict[str, str] = {}
    existing_base_ids = prior_manifest.get("selection", {}).get("base_record_ids")
    if isinstance(existing_base_ids, list) and all(
        isinstance(item, str) for item in existing_base_ids
    ):
        base_record_ids = sorted(existing_base_ids)
    else:
        base_record_ids = sorted(
            str(item.get("record_id", ""))
            for item in raw_records
            if isinstance(item, dict)
        )
    for item in raw_records:
        if not isinstance(item, dict):
            raise ValueError("Prior cohort manifest contains a non-object record")
        record_id = str(item.get("record_id", ""))
        candidate = by_id.get(record_id)
        if candidate is None:
            raise ValueError(f"Prior cohort record is not eligible in the overlay: {record_id}")
        source_hashes = item.get("source_sha256")
        prior_model_hash = (
            source_hashes.get("model_py") if isinstance(source_hashes, dict) else None
        )
        if prior_model_hash != candidate.model_sha256:
            raise ValueError(f"Prior cohort model hash changed: {record_id}")
        selected.append(candidate)
        digest = item.get("selection_digest")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError(f"Prior selection digest is invalid: {record_id}")
        selection_digests[record_id] = digest

    prior_categories = {candidate.category_slug for candidate in selected}
    if len(prior_categories) != len(selected):
        raise ValueError("Prior cohort no longer maps to unique current categories")
    added_categories: list[str] = []
    for category_slug in sorted(by_category):
        if category_slug in prior_categories:
            continue
        candidate = by_category[category_slug][0]
        selected.append(candidate)
        selection_digests[candidate.record_id] = candidate.selection_digest
        added_categories.append(category_slug)
    selected.sort(key=lambda item: (item.category_slug, item.record_id))

    prior_selection = prior_manifest.get("selection")
    if not isinstance(prior_selection, dict):
        prior_selection = {}
    pre_expansion_manifest_sha256 = prior_selection.get(
        "pre_expansion_manifest_sha256", prior_manifest_sha256
    )
    original_selection_manifest_sha256 = prior_selection.get(
        "original_selection_manifest_sha256",
        prior_selection.get("prior_manifest_sha256"),
    )
    base_record_count = int(prior_selection.get("base_record_count", len(raw_records)))
    existing_added_categories = prior_selection.get("added_categories")
    if isinstance(existing_added_categories, list) and all(
        isinstance(item, str) for item in existing_added_categories
    ):
        frozen_added_categories = sorted(existing_added_categories)
    else:
        frozen_added_categories = sorted(added_categories)
    return selected, {
        "description": "preserve every prior cohort record and add one deterministic verified record for each newly hydrated current category",
        "selection_protocol_id": prior_selection.get(
            "selection_protocol_id", prior_manifest.get("protocol_id")
        ),
        "selection_protocol_sha256": prior_selection.get(
            "selection_protocol_sha256", prior_manifest.get("protocol_sha256")
        ),
        "pre_expansion_manifest_sha256": pre_expansion_manifest_sha256,
        "original_selection_manifest_sha256": original_selection_manifest_sha256,
        "base_record_count": base_record_count,
        "base_record_ids": base_record_ids,
        "added_category_count": len(frozen_added_categories),
        "added_categories": frozen_added_categories,
        "expanded_record_count": len(selected),
        "retention_rule": "current checkout dataset collection and primary rating in {4, 5}",
        "hydration_rule": "current hydrated bytes or mirror payload matching the checkout LFS pointer OID and size for every required file",
        "new_category_selection": "minimum SHA256(protocol_id NUL category_slug NUL record_id NUL model_sha256)",
    }, selection_digests


def candidate_manifest(candidate: Candidate) -> dict[str, Any]:
    def rel(path: Path | None) -> str | None:
        return path.relative_to(WORKSPACE_ROOT).as_posix() if path is not None else None

    return {
        "record_id": candidate.record_id,
        "category_slug": candidate.category_slug,
        "primary_rating": candidate.rating,
        "provider": candidate.provider,
        "model_id": candidate.model_id,
        "active_revision_id": candidate.active_revision_id,
        "selection_digest": candidate.selection_digest,
        "source_paths": {
            "record_json": rel(candidate.record_path),
            "dataset_json": rel(candidate.dataset_path),
            "model_py": rel(candidate.model_path),
            "provenance_json": rel(candidate.provenance_path),
            "prompt_txt": rel(candidate.prompt_path),
            "inputs_dir": rel(candidate.inputs_path),
        },
        "source_sha256": {
            "record_json": candidate.record_sha256,
            "model_py": candidate.model_sha256,
            "provenance_json": candidate.provenance_sha256,
            "prompt_txt": candidate.prompt_sha256,
        },
    }


def candidates_from_frozen_overlay_manifest(
    manifest: dict[str, Any],
) -> tuple[list[Candidate], dict[str, Any], dict[str, dict[str, Any]]]:
    raw_records = manifest.get("records")
    if not isinstance(raw_records, list) or not raw_records:
        raise ValueError("Frozen overlay manifest has no records")
    candidates: list[Candidate] = []
    provenance_by_id: dict[str, dict[str, Any]] = {}
    for item in raw_records:
        if not isinstance(item, dict):
            raise ValueError("Frozen overlay manifest contains a non-object record")
        record_id = str(item.get("record_id", ""))
        source_paths = item.get("source_paths")
        source_hashes = item.get("source_sha256")
        provenance = item.get("checkout_provenance")
        if not isinstance(source_paths, dict) or not isinstance(source_hashes, dict):
            raise ValueError(f"Frozen source evidence is missing: {record_id}")
        if not isinstance(provenance, dict) or provenance.get(
            "all_required_files_verified"
        ) is not True:
            raise ValueError(f"Frozen overlay provenance is invalid: {record_id}")

        def source_path(key: str, *, optional: bool = False) -> Path | None:
            value = source_paths.get(key)
            if value is None and optional:
                return None
            if not isinstance(value, str):
                raise ValueError(f"Frozen source path is invalid: {record_id}/{key}")
            return canonical_inside(WORKSPACE_ROOT / value, WORKSPACE_ROOT)

        record_path = source_path("record_json")
        dataset_path = source_path("dataset_json")
        model_path = source_path("model_py")
        provenance_path = source_path("provenance_json")
        prompt_path = source_path("prompt_txt", optional=True)
        inputs_path = source_path("inputs_dir", optional=True)
        assert record_path is not None
        assert dataset_path is not None
        assert model_path is not None
        assert provenance_path is not None
        for key, path in (
            ("record_json", record_path),
            ("model_py", model_path),
            ("provenance_json", provenance_path),
            ("prompt_txt", prompt_path),
        ):
            expected = source_hashes.get(key)
            if path is not None and sha256_file(path) != expected:
                raise ValueError(f"Frozen source hash changed: {record_id}/{key}")
        dataset_evidence = provenance.get("files", {}).get("dataset_json", {})
        if sha256_file(dataset_path) != dataset_evidence.get("payload_sha256"):
            raise ValueError(f"Frozen dataset sidecar hash changed: {record_id}")
        selection_digest = str(item.get("selection_digest", ""))
        if not re.fullmatch(r"[0-9a-f]{64}", selection_digest):
            raise ValueError(f"Frozen selection digest is invalid: {record_id}")
        candidates.append(
            Candidate(
                record_id=record_id,
                category_slug=str(item.get("category_slug", "")),
                rating=int(item.get("primary_rating")),
                model_path=model_path,
                provenance_path=provenance_path,
                prompt_path=prompt_path,
                inputs_path=inputs_path,
                record_path=record_path,
                dataset_path=dataset_path,
                model_sha256=str(source_hashes.get("model_py")),
                provenance_sha256=str(source_hashes.get("provenance_json")),
                prompt_sha256=(
                    str(source_hashes.get("prompt_txt")) if prompt_path is not None else None
                ),
                record_sha256=str(source_hashes.get("record_json")),
                provider=(str(item["provider"]) if item.get("provider") else None),
                model_id=(str(item["model_id"]) if item.get("model_id") else None),
                active_revision_id=str(item.get("active_revision_id", "")),
                selection_digest=selection_digest,
            )
        )
        provenance_by_id[record_id] = provenance
    audit = manifest.get("audit")
    if not isinstance(audit, dict):
        raise ValueError("Frozen overlay manifest audit is invalid")
    return candidates, audit, provenance_by_id


def copy_tree_without_symlinks(source: Path, destination: Path) -> None:
    for directory, dirnames, filenames in os.walk(source, followlinks=False):
        directory_path = Path(directory)
        canonical_inside(directory_path, WORKSPACE_ROOT)
        for name in list(dirnames):
            child = directory_path / name
            if child.is_symlink():
                raise ValueError(f"Input directory contains a symlink: {child}")
        relative = directory_path.relative_to(source)
        target_dir = destination / relative
        target_dir.mkdir(parents=True, exist_ok=True)
        for name in filenames:
            child = directory_path / name
            if child.is_symlink():
                raise ValueError(f"Input directory contains a symlink: {child}")
            canonical_inside(child, WORKSPACE_ROOT)
            shutil.copy2(child, target_dir / name)


def stage_candidate(candidate: Candidate, package_dir: Path) -> Path:
    package_dir.mkdir(parents=True, exist_ok=True)
    model_path = package_dir / "model.py"
    shutil.copy2(candidate.model_path, model_path)
    shutil.copy2(candidate.provenance_path, package_dir / "provenance.json")
    shutil.copy2(candidate.record_path, package_dir / "record.json")
    shutil.copy2(candidate.dataset_path, package_dir / "dataset.json")
    if candidate.prompt_path is not None:
        shutil.copy2(candidate.prompt_path, package_dir / "prompt.txt")
    if candidate.inputs_path is not None:
        copy_tree_without_symlinks(candidate.inputs_path, package_dir / "inputs")
    return model_path


def safe_mesh_path(package_dir: Path, filename: str) -> Path | None:
    raw = filename.strip().replace("\\", "/")
    if raw.startswith("package://"):
        raw = raw[len("package://") :]
    candidate = Path(raw)
    if candidate.is_absolute():
        return None
    try:
        return canonical_inside(package_dir / candidate, package_dir)
    except (FileNotFoundError, RuntimeError, ValueError):
        return None


def finite_positive_values(raw: str, count: int) -> tuple[bool, list[float]]:
    try:
        values = [float(value) for value in raw.split()]
    except ValueError:
        return False, []
    valid = len(values) == count and all(math.isfinite(value) and value > 0 for value in values)
    return valid, values


def evaluate_urdf(urdf_path: Path, placeholder_re: re.Pattern[str]) -> dict[str, Any]:
    root = ET.parse(urdf_path).getroot()
    links = root.findall("link")
    visual_mesh_refs: list[dict[str, Any]] = []
    renderable_links: list[str] = []
    mesh_only_links: list[str] = []
    primitive_only_links: list[str] = []
    mixed_links: list[str] = []
    invalid_geometries: list[dict[str, Any]] = []
    package_dir = urdf_path.parent

    for link in links:
        link_name = str(link.attrib.get("name", "")).strip()
        valid_mesh = False
        valid_primitive = False
        for geometry in link.findall("./visual/geometry"):
            for element in geometry:
                if element.tag == "mesh":
                    filename = str(element.attrib.get("filename", "")).strip()
                    resolved = safe_mesh_path(package_dir, filename) if filename else None
                    exists = (
                        resolved is not None
                        and resolved.is_file()
                        and resolved.stat().st_size > 0
                    )
                    visual_mesh_refs.append(
                        {
                            "link_name": link_name,
                            "filename": filename,
                            "exists_nonempty": exists,
                            "sha256": (
                                sha256_file(resolved)
                                if exists and resolved is not None
                                else None
                            ),
                        }
                    )
                    valid_mesh |= exists
                    if not exists:
                        invalid_geometries.append(
                            {
                                "link_name": link_name,
                                "geometry": "mesh",
                                "value": filename,
                                "reason": "missing, empty, absolute, or package-escaping mesh",
                            }
                        )
                elif element.tag == "box":
                    valid, values = finite_positive_values(str(element.attrib.get("size", "")), 3)
                    valid_primitive |= valid
                    if not valid:
                        invalid_geometries.append(
                            {
                                "link_name": link_name,
                                "geometry": "box",
                                "value": values,
                                "reason": "size must contain three finite positive values",
                            }
                        )
                elif element.tag == "cylinder":
                    radius_ok, radius = finite_positive_values(
                        str(element.attrib.get("radius", "")), 1
                    )
                    length_ok, length = finite_positive_values(
                        str(element.attrib.get("length", "")), 1
                    )
                    valid = radius_ok and length_ok
                    valid_primitive |= valid
                    if not valid:
                        invalid_geometries.append(
                            {
                                "link_name": link_name,
                                "geometry": "cylinder",
                                "value": {"radius": radius, "length": length},
                                "reason": "radius and length must be finite and positive",
                            }
                        )
                elif element.tag == "sphere":
                    valid, values = finite_positive_values(
                        str(element.attrib.get("radius", "")), 1
                    )
                    valid_primitive |= valid
                    if not valid:
                        invalid_geometries.append(
                            {
                                "link_name": link_name,
                                "geometry": "sphere",
                                "value": values,
                                "reason": "radius must be finite and positive",
                            }
                        )
                else:
                    invalid_geometries.append(
                        {
                            "link_name": link_name,
                            "geometry": element.tag,
                            "value": dict(element.attrib),
                            "reason": "unsupported visual geometry",
                        }
                    )
        if valid_mesh or valid_primitive:
            renderable_links.append(link_name)
            if valid_mesh and valid_primitive:
                mixed_links.append(link_name)
            elif valid_mesh:
                mesh_only_links.append(link_name)
            else:
                primitive_only_links.append(link_name)

    named_links = [
        name for name in renderable_links if name and placeholder_re.fullmatch(name) is None
    ]
    return {
        "urdf_link_count": len(links),
        "visual_mesh_reference_count": len(visual_mesh_refs),
        "renderable_visual_part_count": len(renderable_links),
        "named_renderable_visual_part_count": len(named_links),
        "renderable_visual_link_names": renderable_links,
        "mesh_only_link_count": len(mesh_only_links),
        "primitive_only_link_count": len(primitive_only_links),
        "mixed_link_count": len(mixed_links),
        "links_with_mesh_geometry": len(mesh_only_links) + len(mixed_links),
        "links_with_primitive_geometry": len(primitive_only_links) + len(mixed_links),
        "mesh_only_link_names": mesh_only_links,
        "primitive_only_link_names": primitive_only_links,
        "mixed_link_names": mixed_links,
        "placeholder_link_names": [
            name for name in renderable_links if name not in named_links
        ],
        "invalid_visual_geometries": invalid_geometries,
        "visual_mesh_references": visual_mesh_refs,
        "evaluable": bool(renderable_links),
    }


def run_case(
    candidate: Candidate,
    output_dir: Path,
    placeholder_pattern: str,
    protocol_sha256: str,
    script_sha256: str,
) -> dict[str, Any]:
    case_dir = output_dir / "cases" / candidate.record_id
    result_path = case_dir / "result.json"
    if result_path.is_file():
        try:
            previous = read_json(result_path)
            if (
                previous.get("status") in TERMINAL_STATUSES
                and previous.get("source_model_sha256") == candidate.model_sha256
                and previous.get("protocol_sha256") == protocol_sha256
                and previous.get("harness_sha256") == script_sha256
            ):
                previous["resumed"] = True
                return previous
        except (json.JSONDecodeError, OSError, ValueError):
            pass

    if case_dir.exists():
        shutil.rmtree(case_dir)
    package_dir = case_dir / "package"
    started = time.perf_counter()
    base: dict[str, Any] = {
        "record_id": candidate.record_id,
        "category_slug": candidate.category_slug,
        "source_model_sha256": candidate.model_sha256,
        "protocol_sha256": protocol_sha256,
        "harness_sha256": script_sha256,
        "rating": candidate.rating,
        "representation": "native Articraft URDF",
        "part_unit": "mesh-bearing URDF link",
        "resumed": False,
    }
    try:
        staged_model = stage_candidate(candidate, package_dir)
        report = compile_urdf_report_maybe_timeout(
            staged_model,
            sdk_package="sdk",
            run_checks=False,
            target="visual",
            rewrite_visual_glb=False,
        )
        urdf_path = package_dir / "model.urdf"
        urdf_path.write_text(report.urdf_xml, encoding="utf-8")
        direct = evaluate_urdf(
            urdf_path,
            re.compile(placeholder_pattern, flags=re.IGNORECASE),
        )
        status = "PASS" if direct["evaluable"] else "PACKAGE_ERROR"
        result = {
            **base,
            "status": status,
            "elapsed_seconds": time.perf_counter() - started,
            "compile_target": "visual",
            "compile_checks": False,
            "warnings": list(report.warnings),
            "signal_bundle": report.signal_bundle.to_dict(),
            "urdf_sha256": sha256_file(urdf_path),
            "direct_naming": direct,
            "error": (
                None
                if status == "PASS"
                else "URDF has no link with valid renderable visual geometry"
            ),
        }
    except TimeoutError as exc:
        result = {
            **base,
            "status": "TIMEOUT",
            "elapsed_seconds": time.perf_counter() - started,
            "direct_naming": None,
            "error": str(exc),
        }
    except RuntimeError as exc:
        result = {
            **base,
            "status": "COMPILE_ERROR",
            "elapsed_seconds": time.perf_counter() - started,
            "direct_naming": None,
            "error": str(exc),
            "remote_error_type": getattr(exc, "remote_error_type", None),
        }
    except Exception as exc:
        result = {
            **base,
            "status": "HARNESS_ERROR",
            "elapsed_seconds": time.perf_counter() - started,
            "direct_naming": None,
            "error": f"{type(exc).__name__}: {exc}",
        }
    dump_json(result_path, result)
    return result


def reevaluate_existing_case(
    candidate: Candidate,
    output_dir: Path,
    placeholder_pattern: str,
    protocol_sha256: str,
    script_sha256: str,
) -> dict[str, Any]:
    case_dir = output_dir / "cases" / candidate.record_id
    result_path = canonical_inside(case_dir / "result.json", REPO_ROOT)
    urdf_path = canonical_inside(case_dir / "package" / "model.urdf", REPO_ROOT)
    previous = read_json(result_path)
    if previous.get("source_model_sha256") != candidate.model_sha256:
        raise ValueError(f"Source model hash changed for {candidate.record_id}")
    previous_urdf_hash = previous.get("urdf_sha256")
    current_urdf_hash = sha256_file(urdf_path)
    if previous_urdf_hash != current_urdf_hash:
        raise ValueError(f"Existing URDF hash changed for {candidate.record_id}")

    direct = evaluate_urdf(
        urdf_path,
        re.compile(placeholder_pattern, flags=re.IGNORECASE),
    )
    status = "PASS" if direct["evaluable"] else "PACKAGE_ERROR"
    result = {
        **previous,
        "record_id": candidate.record_id,
        "category_slug": candidate.category_slug,
        "source_model_sha256": candidate.model_sha256,
        "rating": candidate.rating,
        "status": status,
        "protocol_sha256": protocol_sha256,
        "harness_sha256": script_sha256,
        "part_unit": "URDF link with valid renderable visual geometry",
        "direct_naming": direct,
        "error": (
            None
            if status == "PASS"
            else "URDF has no link with valid renderable visual geometry"
        ),
        "resumed": False,
    }
    dump_json(result_path, result)
    return result


def run_or_reevaluate_case(
    candidate: Candidate,
    output_dir: Path,
    placeholder_pattern: str,
    protocol_sha256: str,
    script_sha256: str,
) -> dict[str, Any]:
    case_dir = output_dir / "cases" / candidate.record_id
    result_path = case_dir / "result.json"
    urdf_path = case_dir / "package" / "model.urdf"
    if result_path.is_file() and urdf_path.is_file():
        try:
            return reevaluate_existing_case(
                candidate,
                output_dir,
                placeholder_pattern,
                protocol_sha256,
                script_sha256,
            )
        except (json.JSONDecodeError, OSError, ValueError):
            pass
    return run_case(
        candidate,
        output_dir,
        placeholder_pattern,
        protocol_sha256,
        script_sha256,
    )


def resume_missing_case(
    candidate: Candidate,
    output_dir: Path,
    placeholder_pattern: str,
    protocol_sha256: str,
    script_sha256: str,
) -> dict[str, Any]:
    case_dir = output_dir / "cases" / candidate.record_id
    result_path = case_dir / "result.json"
    urdf_path = case_dir / "package" / "model.urdf"
    if result_path.is_file() and urdf_path.is_file():
        previous = read_json(result_path)
        if previous.get("status") not in TERMINAL_STATUSES:
            raise ValueError(f"Existing result is not terminal: {candidate.record_id}")
        if previous.get("source_model_sha256") != candidate.model_sha256:
            raise ValueError(f"Source model hash changed for {candidate.record_id}")
        if previous.get("urdf_sha256") != sha256_file(urdf_path):
            raise ValueError(f"Existing URDF hash changed for {candidate.record_id}")
        return previous
    return run_case(
        candidate,
        output_dir,
        placeholder_pattern,
        protocol_sha256,
        script_sha256,
    )


def quantile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def bootstrap_mean_ci(values: list[float], seed: int, resamples: int) -> list[float] | None:
    if not values:
        return None
    import random

    rng = random.Random(seed)
    means = [
        statistics.fmean(values[rng.randrange(len(values))] for _ in values)
        for _ in range(resamples)
    ]
    return [quantile(means, 0.025), quantile(means, 0.975)]  # type: ignore[list-item]


def build_summary(
    results: list[dict[str, Any]],
    protocol: dict[str, Any],
    protocol_sha256: str,
    manifest_sha256: str,
    selection_metadata: dict[str, Any],
) -> dict[str, Any]:
    status_counts = Counter(str(result.get("status")) for result in results)
    evaluable = [
        result
        for result in results
        if result.get("status") == "PASS" and isinstance(result.get("direct_naming"), dict)
    ]
    parts = [
        float(result["direct_naming"]["renderable_visual_part_count"])
        for result in evaluable
    ]
    named = [
        int(result["direct_naming"]["named_renderable_visual_part_count"])
        for result in evaluable
    ]
    total = [
        int(result["direct_naming"]["renderable_visual_part_count"])
        for result in evaluable
    ]
    mesh_only = [int(result["direct_naming"]["mesh_only_link_count"]) for result in evaluable]
    primitive_only = [
        int(result["direct_naming"]["primitive_only_link_count"]) for result in evaluable
    ]
    mixed = [int(result["direct_naming"]["mixed_link_count"]) for result in evaluable]
    invalid_geometry = [
        len(result["direct_naming"]["invalid_visual_geometries"]) for result in evaluable
    ]
    placeholder_pairs = [
        (str(result["record_id"]), str(name))
        for result in evaluable
        for name in result["direct_naming"]["placeholder_link_names"]
    ]
    elapsed = [float(result.get("elapsed_seconds", 0.0)) for result in results]
    bootstrap = protocol.get("bootstrap", {})
    bootstrap_seed = int(bootstrap.get("seed", 260811002))
    return {
        "status": "COMPLETE" if len(results) > 0 else "EMPTY",
        "method": "Articraft",
        "cohort": "one deterministic retained hydrated record per eligible category",
        "cohort_selection_lineage": selection_metadata,
        "protocol_id": protocol.get("protocol_id"),
        "protocol_sha256": protocol_sha256,
        "cohort_manifest_sha256": manifest_sha256,
        "representation": "native Articraft URDF",
        "part_unit": "one URDF link with at least one valid renderable visual geometry",
        "artifact_coverage": {
            "requested": len(results),
            "evaluable": len(evaluable),
            "rate": len(evaluable) / len(results) if results else None,
            "status_counts": dict(sorted(status_counts.items())),
        },
        "direct_metrics": {
            "parts_per_asset_mean": statistics.fmean(parts) if parts else None,
            "parts_per_asset_median": statistics.median(parts) if parts else None,
            "parts_per_asset_bootstrap_95_ci": bootstrap_mean_ci(
                parts, bootstrap_seed, int(bootstrap.get("resamples", 10000))
            ),
            "renderable_visual_parts_total": sum(total),
            "named_renderable_visual_parts_total": sum(named),
            "nameability": sum(named) / sum(total) if sum(total) else None,
            "protocol_placeholder_audit": {
                "placeholder_parts": len(placeholder_pairs),
                "placeholder_rate": len(placeholder_pairs) / sum(total) if sum(total) else None,
                "affected_assets": len({record_id for record_id, _ in placeholder_pairs}),
                "name_counts": dict(
                    sorted(Counter(name for _, name in placeholder_pairs).items())
                ),
                "records": [
                    {"record_id": record_id, "link_name": name}
                    for record_id, name in sorted(placeholder_pairs)
                ],
                "interpretation": "The shared placeholder regex detects opaque/generic identifiers only; names outside it are non-placeholder, not proven semantically correct.",
            },
            "geometry_link_composition": {
                "mesh_only_links": sum(mesh_only),
                "primitive_only_links": sum(primitive_only),
                "mixed_links": sum(mixed),
                "links_with_mesh_geometry": sum(mesh_only) + sum(mixed),
                "links_with_primitive_geometry": sum(primitive_only) + sum(mixed),
                "invalid_visual_geometries": sum(invalid_geometry),
            },
        },
        "semantic_metrics": {
            "semantic_precision": None,
            "semantic_recall": None,
            "naming_richness": None,
            "functional_core_coverage": None,
            "instance_discriminability": None,
            "over_segmentation_rate": None,
            "reason": "No frozen output-independent role gold and no three independent blind judge verdicts for this Articraft cohort.",
        },
        "cross_seed_consistency": {
            "value": None,
            "reason": "Articraft is evaluated as a per-asset method; no official frozen reusable seed interface was found in the audited records.",
        },
        "execution": {
            "elapsed_seconds_total_case_sum": sum(elapsed),
            "elapsed_seconds_mean": statistics.fmean(elapsed) if elapsed else None,
            "elapsed_seconds_median": statistics.median(elapsed) if elapsed else None,
            "elapsed_seconds_p95": quantile(elapsed, 0.95),
        },
        "limitations": [
            "This is a local public retained-category cohort, not a common prompt-matched authoring rerun.",
            "Only direct renderable-visual URDF-link Parts and Nameability are scored; readable source names do not prove semantic correctness.",
            "URDF-link counts are not labeled as GLB-node counts because no converter-preservation audit was run for this cohort.",
            "Visual compilation verifies naming-evaluable packages, not full authored tests or physical QC.",
        ],
    }


def render_report(summary: dict[str, Any], audit: dict[str, Any]) -> str:
    coverage = summary["artifact_coverage"]
    direct = summary["direct_metrics"]
    composition = direct["geometry_link_composition"]
    placeholders = direct["protocol_placeholder_audit"]
    ci = direct["parts_per_asset_bootstrap_95_ci"]
    ci_text = "N/A" if ci is None else f"[{ci[0]:.3f}, {ci[1]:.3f}]"
    selection = summary["cohort_selection_lineage"]
    prior_manifest_hash = selection.get(
        "pre_expansion_manifest_sha256",
        selection.get("prior_manifest_sha256", "N/A"),
    )
    expansion_text = ""
    if "expanded_record_count" in selection:
        expansion_text = (
            f" The exact {selection['base_record_count']}-record base is retained; "
            f"{selection['added_category_count']} newly hydrated categories are added."
        )
    return f"""# Articraft Naming Baseline v1.1

Status: **{summary['status']}**

## Frozen cohort

- Selection protocol: `{selection.get('selection_protocol_id')}` (`{selection.get('selection_protocol_sha256')}`).
- Pre-expansion frozen manifest SHA-256: `{prior_manifest_hash}`.{expansion_text}
- Eligible pool: {audit['counts'].get('eligible_hydrated_retained_records', 0)} hydrated retained records across {audit['eligible_category_count']} categories.
- Requested cohort: {coverage['requested']} records.
- Representation/unit: native Articraft URDF; one link with at least one valid renderable visual geometry.
- Shared protocol: `{summary['protocol_id']}` (`{summary['protocol_sha256']}`).
- Cohort manifest SHA-256: `{summary['cohort_manifest_sha256']}`.

## Direct results

- Artifact coverage: {coverage['evaluable']}/{coverage['requested']} = {coverage['rate']:.6f}.
- Status counts: `{json.dumps(coverage['status_counts'], sort_keys=True)}`.
- Parts: {direct['parts_per_asset_mean']:.3f} renderable-visual URDF links/asset; median {direct['parts_per_asset_median']:.3f}; 95% bootstrap CI {ci_text}; total {direct['renderable_visual_parts_total']}.
- Named / Nameability: {direct['named_renderable_visual_parts_total']}/{direct['renderable_visual_parts_total']} = {direct['nameability']:.6f}.
- Protocol-placeholder audit: {placeholders['placeholder_parts']}/{direct['renderable_visual_parts_total']} across {placeholders['affected_assets']} assets; names `{json.dumps(placeholders['name_counts'], sort_keys=True)}`. Non-placeholder does not certify semantic correctness.
- Geometry link composition: mesh-only {composition['mesh_only_links']}; primitive-only {composition['primitive_only_links']}; mixed {composition['mixed_links']}; invalid visual geometries {composition['invalid_visual_geometries']}.

## Fail-closed fields

Semantic Precision, Semantic Recall, Naming Richness, Functional Core Coverage,
Instance Discriminability, and Over-Segmentation Rate are **N/A**. This cohort has
no frozen output-independent role gold and no completed three-judge blind verdicts.
Readable source/link names are not used to certify semantic correctness.

Cross-Seed Consistency is **N/A** because Articraft is evaluated as a per-asset
method and the audited records expose no official frozen reusable seed interface.

## Scope

This is a local retained-category supplementary cohort, not the common hidden
prompt-matched authoring comparison. Visual compile was used only to produce
naming-evaluable native URDF packages; it is not full physical QC. URDF-link
counts are not relabeled as GLB-node counts without a converter-preservation audit.
"""


def build_self_check(
    output_dir: Path,
    protocol_path: Path,
    expected_protocol_sha256: str,
) -> dict[str, Any]:
    manifest = read_json(output_dir / "cohort_manifest.json")
    summary = read_json(output_dir / "summary.json")
    run_metadata = read_json(output_dir / "run_metadata.json")
    records = [
        json.loads(line)
        for line in (output_dir / "records.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    manifest_records = manifest.get("records")
    if not isinstance(manifest_records, list):
        raise ValueError("Manifest records must be a list")
    manifest_by_id = {str(item["record_id"]): item for item in manifest_records}

    record_ids = [str(item["record_id"]) for item in records]
    category_slugs = [str(item["category_slug"]) for item in records]
    unique_records = len(set(record_ids))
    unique_categories = len(set(category_slugs))
    expected_records = int(summary["artifact_coverage"]["requested"])
    all_pass = all(item.get("status") == "PASS" for item in records)
    all_evaluable = all(
        isinstance(item.get("direct_naming"), dict)
        and item["direct_naming"].get("evaluable") is True
        for item in records
    )

    direct = summary["direct_metrics"]
    composition = direct["geometry_link_composition"]
    placeholders = direct["protocol_placeholder_audit"]
    parts = int(direct["renderable_visual_parts_total"])
    named = int(direct["named_renderable_visual_parts_total"])
    composition_sum = (
        int(composition["mesh_only_links"])
        + int(composition["primitive_only_links"])
        + int(composition["mixed_links"])
    )
    named_placeholder_sum = named + int(placeholders["placeholder_parts"])

    live_source_hash_matches = 0
    staged_source_hash_matches = 0
    urdf_hash_matches = 0
    mesh_reference_hash_matches = 0
    mesh_reference_count = 0
    overlay_provenance_verified = 0
    for result in records:
        record_id = str(result["record_id"])
        manifest_item = manifest_by_id[record_id]
        checkout_provenance = manifest_item.get("checkout_provenance")
        if isinstance(checkout_provenance, dict) and checkout_provenance.get(
            "all_required_files_verified"
        ) is True:
            overlay_provenance_verified += 1
        source = manifest_item["source_paths"]
        hashes = manifest_item["source_sha256"]
        live_model = canonical_inside(
            WORKSPACE_ROOT / source["model_py"], WORKSPACE_ROOT
        )
        if sha256_file(live_model) == hashes["model_py"]:
            live_source_hash_matches += 1
        package_dir = canonical_inside(
            output_dir / "cases" / record_id / "package", REPO_ROOT
        )
        if sha256_file(package_dir / "model.py") == hashes["model_py"]:
            staged_source_hash_matches += 1
        urdf_path = canonical_inside(package_dir / "model.urdf", REPO_ROOT)
        if sha256_file(urdf_path) == result["urdf_sha256"]:
            urdf_hash_matches += 1
        for mesh_ref in result["direct_naming"]["visual_mesh_references"]:
            mesh_reference_count += 1
            mesh_path = safe_mesh_path(package_dir, str(mesh_ref["filename"]))
            if (
                mesh_path is not None
                and mesh_path.is_file()
                and mesh_path.stat().st_size > 0
                and sha256_file(mesh_path) == mesh_ref["sha256"]
            ):
                mesh_reference_hash_matches += 1

    output_symlink_count = 0
    for directory, dirnames, filenames in os.walk(output_dir, followlinks=False):
        base = Path(directory)
        output_symlink_count += sum((base / name).is_symlink() for name in dirnames)
        output_symlink_count += sum((base / name).is_symlink() for name in filenames)

    semantic = summary["semantic_metrics"]
    semantic_null = all(
        value is None for key, value in semantic.items() if key != "reason"
    )
    protocol_hash = sha256_file(protocol_path)
    artifact_names = [
        "cohort_manifest.json",
        "records.jsonl",
        "summary.json",
        "report.md",
        "run_metadata.json",
    ]
    if (output_dir / "source_audit.json").is_file():
        artifact_names.append("source_audit.json")
    artifact_hashes = {name: sha256_file(output_dir / name) for name in artifact_names}
    reproduction_digest = sha256_bytes(
        json.dumps(artifact_hashes, sort_keys=True, separators=(",", ":")).encode("utf-8")
    )
    checks = {
        "record_count_matches_requested": len(records) == expected_records,
        "unique_record_count_matches_requested": unique_records == expected_records,
        "unique_category_count_matches_requested": unique_categories == expected_records,
        "manifest_record_set_matches": set(record_ids) == set(manifest_by_id),
        "all_pass": all_pass,
        "all_evaluable": all_evaluable,
        "composition_sum_matches_parts": composition_sum == parts,
        "named_plus_placeholder_matches_parts": named_placeholder_sum == parts,
        "invalid_visual_geometries_zero": composition["invalid_visual_geometries"] == 0,
        "live_source_model_hashes_match_manifest": (
            live_source_hash_matches == expected_records
        ),
        "staged_source_model_hashes_match_manifest": (
            staged_source_hash_matches == expected_records
        ),
        "urdf_hashes_match": urdf_hash_matches == expected_records,
        "mesh_reference_hashes_match": mesh_reference_hash_matches == mesh_reference_count,
        "output_symlink_count_zero": output_symlink_count == 0,
        "semantic_fields_null": semantic_null,
        "cross_seed_null": summary["cross_seed_consistency"]["value"] is None,
        "protocol_hash_current": protocol_hash == expected_protocol_sha256,
        "run_metadata_hashes_match": all(
            run_metadata.get(key) == artifact_hashes[name]
            for name, key in (
                ("records.jsonl", "records_jsonl_sha256"),
                ("summary.json", "summary_sha256"),
                ("report.md", "report_sha256"),
            )
        ),
    }
    selection = manifest.get("selection")
    if isinstance(selection, dict) and "expanded_record_count" in selection:
        base_ids = selection.get("base_record_ids")
        added_categories = selection.get("added_categories")
        checks.update(
            {
                "base_record_count_123": (
                    isinstance(base_ids, list)
                    and len(base_ids) == 123
                    and len(set(base_ids)) == 123
                ),
                "all_base_record_ids_retained": (
                    isinstance(base_ids, list) and set(base_ids).issubset(record_ids)
                ),
                "added_category_count_119": (
                    isinstance(added_categories, list)
                    and len(added_categories) == 119
                    and len(set(added_categories)) == 119
                ),
                "added_categories_present_exactly_once": (
                    isinstance(added_categories, list)
                    and all(category_slugs.count(item) == 1 for item in added_categories)
                ),
                "expanded_record_count_242": expected_records == 242,
                "overlay_required_file_provenance_verified": (
                    overlay_provenance_verified == expected_records
                ),
                "source_audit_hash_matches_manifest": (
                    (output_dir / "source_audit.json").is_file()
                    and manifest.get("source_audit_sha256")
                    == sha256_file(output_dir / "source_audit.json")
                ),
                "source_audit_hash_matches_run_metadata": (
                    (output_dir / "source_audit.json").is_file()
                    and run_metadata.get("source_audit_sha256")
                    == sha256_file(output_dir / "source_audit.json")
                ),
            }
        )
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise AssertionError(f"Self-check failed: {', '.join(failed)}")
    return {
        "status": "PASS",
        "protocol_id": summary["protocol_id"],
        "protocol_sha256": protocol_hash,
        "checks": checks,
        "counts": {
            "records": len(records),
            "unique_records": unique_records,
            "unique_categories": unique_categories,
            "renderable_visual_parts": parts,
            "named_parts": named,
            "protocol_placeholder_parts": int(placeholders["placeholder_parts"]),
            "mesh_only_links": int(composition["mesh_only_links"]),
            "primitive_only_links": int(composition["primitive_only_links"]),
            "mixed_links": int(composition["mixed_links"]),
            "invalid_visual_geometries": int(composition["invalid_visual_geometries"]),
            "live_source_hash_matches": live_source_hash_matches,
            "staged_source_hash_matches": staged_source_hash_matches,
            "urdf_hash_matches": urdf_hash_matches,
            "mesh_reference_count": mesh_reference_count,
            "mesh_reference_hash_matches": mesh_reference_hash_matches,
            "output_symlink_count": output_symlink_count,
            "overlay_provenance_verified_records": overlay_provenance_verified,
        },
        "artifact_sha256": artifact_hashes,
        "reproduction_digest": reproduction_digest,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--max-categories", type=int, default=0)
    parser.add_argument(
        "--lfs-mirror-records",
        type=Path,
        default=None,
        help=(
            "Resolve checkout LFS pointers from this workspace-local records mirror; "
            "every payload must match the pointer OID and size."
        ),
    )
    parser.add_argument(
        "--evaluate-existing",
        action="store_true",
        help="Re-evaluate the exact frozen cohort's existing URDF packages without compiling.",
    )
    parser.add_argument(
        "--resume-missing-only",
        action="store_true",
        help="Preserve verified terminal case results byte-for-byte and compile only missing cases.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    canonical_inside(REPO_ROOT, REPO_ROOT)
    canonical_inside(WORKSPACE_ROOT, WORKSPACE_ROOT)
    canonical_inside(ARTICRAFT_ROOT, REPO_ROOT)
    canonical_inside(RECORDS_ROOT, REPO_ROOT)
    protocol_path = canonical_inside(args.protocol, REPO_ROOT)
    output_dir = canonical_inside(args.output_dir, REPO_ROOT, must_exist=False)
    output_dir.mkdir(parents=True, exist_ok=True)
    protocol = read_json(protocol_path)
    protocol_sha256 = sha256_file(protocol_path)
    script_sha256 = sha256_file(SCRIPT_PATH)
    protocol_id = str(protocol.get("protocol_id", ""))
    if not protocol_id:
        raise ValueError("Protocol is missing protocol_id")
    placeholder_pattern = str(protocol["nameability"]["placeholder_regex"])
    os.environ["URDF_COMPILE_TIMEOUT_SECONDS"] = str(float(args.timeout))
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"

    prior_manifest_path = output_dir / "cohort_manifest.json"
    selection_digests: dict[str, str] = {}
    provenance_by_id: dict[str, dict[str, Any]] = {}
    source_audit: dict[str, Any] | None = None
    preserve_prior_manifest = False
    if args.lfs_mirror_records is not None:
        if int(args.max_categories) != 0:
            raise ValueError(
                "--lfs-mirror-records cannot be combined with --max-categories"
            )
        if args.evaluate_existing and args.resume_missing_only:
            raise ValueError(
                "--evaluate-existing and --resume-missing-only are mutually exclusive"
            )
        mirror_records_root = canonical_inside(args.lfs_mirror_records, WORKSPACE_ROOT)
        prior_manifest_path = canonical_inside(prior_manifest_path, REPO_ROOT)
        prior_manifest_sha256 = sha256_file(prior_manifest_path)
        prior_manifest = read_json(prior_manifest_path)
        if args.resume_missing_only or args.evaluate_existing:
            source_audit_path = canonical_inside(output_dir / "source_audit.json", REPO_ROOT)
            expected_source_audit_hash = prior_manifest.get("source_audit_sha256")
            if sha256_file(source_audit_path) != expected_source_audit_hash:
                raise ValueError("Frozen source audit hash changed")
            source_audit = read_json(source_audit_path)
            candidates, audit, provenance_by_id = candidates_from_frozen_overlay_manifest(
                prior_manifest
            )
            selected = sorted(
                candidates, key=lambda item: (item.category_slug, item.record_id)
            )
            selection_metadata = prior_manifest.get("selection")
            if not isinstance(selection_metadata, dict):
                raise ValueError("Frozen selection metadata is invalid")
            selection_digests = {
                candidate.record_id: candidate.selection_digest for candidate in selected
            }
            preserve_prior_manifest = True
        else:
            candidates, audit, provenance_by_id = audit_overlay_candidates(
                protocol_id,
                RECORDS_ROOT,
                mirror_records_root,
            )
            selected, selection_metadata, selection_digests = select_expanded_cohort(
                candidates,
                prior_manifest,
                prior_manifest_sha256,
            )
        selected_resolution_counts: Counter[str] = Counter()
        selected_lfs_oids: set[str] = set()
        for candidate in selected:
            evidence = provenance_by_id[candidate.record_id]
            for item in evidence["files"].values():
                selected_resolution_counts[str(item.get("status"))] += 1
                lfs_oid = item.get("lfs_oid")
                if isinstance(lfs_oid, str):
                    selected_lfs_oids.add(lfs_oid)
        if not preserve_prior_manifest:
            source_audit = {
                "schema_version": 1,
                "mode": "current_checkout_with_oid_and_size_verified_lfs_overlay",
                "checkout_git_commit": git_commit(ARTICRAFT_ROOT),
                "checkout_records_root": RECORDS_ROOT.relative_to(WORKSPACE_ROOT).as_posix(),
                "mirror_records_root": mirror_records_root.relative_to(WORKSPACE_ROOT).as_posix(),
                "provenance_boundary": (
                    "Current checkout metadata is canonical. Mirror metadata is never "
                    "used to select or label a record; mirror bytes are accepted only "
                    "when the current checkout file is an LFS pointer and payload SHA-256 "
                    "and size match that pointer."
                ),
                "pool_audit": audit,
                "selected_cohort": {
                    "records": len(selected),
                    "base_records_retained": selection_metadata["base_record_count"],
                    "new_categories_added": selection_metadata["added_category_count"],
                    "required_file_resolution_counts": dict(
                        sorted(selected_resolution_counts.items())
                    ),
                    "unique_verified_lfs_oids": len(selected_lfs_oids),
                },
                "semantic_evidence": {
                    "configured_output_independent_gold_paths": [],
                    "configured_three_blind_judge_verdict_paths": [],
                    "metric_value": None,
                    "reason": (
                        "No output-independent role gold and no completed three-judge "
                        "blind verdict set is present for this frozen output cohort."
                    ),
                },
                "seed_interface_evidence": {
                    "official_frozen_reusable_seed_interface": None,
                    "metric_value": None,
                    "reason": (
                        "No official frozen reusable seed interface is documented by the "
                        "audited current records; Articraft remains a per-asset method."
                    ),
                },
            }
            dump_json(output_dir / "source_audit.json", source_audit)
    else:
        candidates, audit = audit_candidates(protocol_id)
    if args.evaluate_existing and args.lfs_mirror_records is None:
        if int(args.max_categories) != 0:
            raise ValueError("--max-categories cannot be used with --evaluate-existing")
        prior_manifest_path = canonical_inside(prior_manifest_path, REPO_ROOT)
        prior_manifest_sha256 = sha256_file(prior_manifest_path)
        prior_manifest = read_json(prior_manifest_path)
        selected, selection_metadata, selection_digests = select_existing_cohort(
            candidates,
            prior_manifest,
            prior_manifest_sha256,
        )
    elif args.lfs_mirror_records is None:
        selected = select_cohort(candidates, int(args.max_categories))
        selection_metadata = {
            "description": "one record per category minimizing SHA256(protocol_id NUL category_slug NUL record_id NUL model_sha256)",
            "selection_protocol_id": protocol_id,
            "selection_protocol_sha256": protocol_sha256,
            "retention_rule": "canonical dataset collection and primary rating in {4, 5}",
            "hydration_rule": "parseable record.json with active model.py and provenance.json, matching declared model hash",
            "max_categories": int(args.max_categories),
        }
    if not selected:
        raise RuntimeError("No eligible Articraft records found")
    manifest = prior_manifest if preserve_prior_manifest else {
        "schema_version": 1,
        "method": "Articraft",
        "protocol_id": protocol_id,
        "protocol_sha256": protocol_sha256,
        "articraft_git_commit": git_commit(ARTICRAFT_ROOT),
        "selection": selection_metadata,
        "audit": audit,
        "source_audit_sha256": (
            sha256_file(output_dir / "source_audit.json")
            if source_audit is not None
            else None
        ),
        "records": [
            {
                **candidate_manifest(candidate),
                "selection_digest": selection_digests.get(
                    candidate.record_id, candidate.selection_digest
                ),
                **(
                    {"checkout_provenance": provenance_by_id[candidate.record_id]}
                    if candidate.record_id in provenance_by_id
                    else {}
                ),
            }
            for candidate in selected
        ],
    }
    manifest_path = output_dir / "cohort_manifest.json"
    if not preserve_prior_manifest:
        dump_json(manifest_path, manifest)
    manifest_hash = sha256_file(manifest_path)

    results: list[dict[str, Any]] = []
    if args.resume_missing_only:
        if args.lfs_mirror_records is None:
            raise ValueError("--resume-missing-only requires --lfs-mirror-records")
        case_runner = resume_missing_case
    elif args.evaluate_existing:
        case_runner = reevaluate_existing_case
    elif args.lfs_mirror_records is not None:
        case_runner = run_or_reevaluate_case
    else:
        case_runner = run_case
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, int(args.workers))) as pool:
        future_map = {
            pool.submit(
                case_runner,
                candidate,
                output_dir,
                placeholder_pattern,
                protocol_sha256,
                script_sha256,
            ): candidate
            for candidate in selected
        }
        completed = 0
        for future in concurrent.futures.as_completed(future_map):
            candidate = future_map[future]
            result = future.result()
            results.append(result)
            completed += 1
            print(
                f"[{completed}/{len(selected)}] {candidate.category_slug} "
                f"{candidate.record_id}: {result.get('status')}",
                flush=True,
            )

    results.sort(key=lambda item: (str(item.get("category_slug")), str(item.get("record_id"))))
    records_path = output_dir / "records.jsonl"
    records_path.write_text(
        "".join(json.dumps(item, sort_keys=True) + "\n" for item in results),
        encoding="utf-8",
    )
    summary = build_summary(
        results,
        protocol,
        protocol_sha256,
        manifest_hash,
        selection_metadata,
    )
    dump_json(output_dir / "summary.json", summary)
    (output_dir / "report.md").write_text(render_report(summary, audit), encoding="utf-8")
    dump_json(
        output_dir / "run_metadata.json",
        {
            "argv": sys.argv,
            "mode": (
                "resume_missing_verified_lfs_overlay"
                if args.resume_missing_only
                else "evaluate_frozen_verified_lfs_overlay"
                if args.evaluate_existing and args.lfs_mirror_records is not None
                else
                "expand_verified_lfs_overlay"
                if args.lfs_mirror_records is not None
                else "evaluate_existing"
                if args.evaluate_existing
                else "compile_and_evaluate"
            ),
            "workers": max(1, int(args.workers)),
            "timeout_seconds": float(args.timeout),
            "python": sys.version,
            "script_sha256": script_sha256,
            "records_jsonl_sha256": sha256_file(records_path),
            "summary_sha256": sha256_file(output_dir / "summary.json"),
            "report_sha256": sha256_file(output_dir / "report.md"),
            "source_audit_sha256": (
                sha256_file(output_dir / "source_audit.json")
                if source_audit is not None
                else None
            ),
        },
    )
    dump_json(
        output_dir / "self_check.json",
        build_self_check(output_dir, protocol_path, protocol_sha256),
    )
    print(json.dumps(summary["artifact_coverage"], indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
