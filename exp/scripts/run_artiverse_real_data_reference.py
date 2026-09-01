#!/usr/bin/env python3
"""Freeze and evaluate Artiverse as a pre-release real-data reference.

The primary panel is a four-category matched overlap with Table 3. A separate
five-category sensitivity maps coffee_table to table and is never mixed
with the primary result. Selection uses only SHA256(raw_category/source/model_id)
from the published chunk manifest. Frozen failures are retained without replacement.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import random
import shutil
from typing import Any, Iterable

from hierarchy_extended_metrics import (
    MOVABLE_TYPES,
    aggregate as aggregate_structure,
    analyze_urdf,
    topology_consistency,
)
from partnet_hierarchy_correctness import (
    aggregate as aggregate_alignment,
    evaluate_urdf,
    load_protocol,
)


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_ARTIVERSE = EXP_ROOT / "artiverse"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/artiverse_reference"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_REFERENCE_PROTOCOL = EXP_ROOT / "reference/artiverse_hierarchy_reference_v1.json"

# Fixed before asset-content inspection. Generic or ambiguous storage classes such
# as storage_box, storage_bench, storage_chest, desk, and coffee_table are excluded
# from the primary panel. Coffee table appears only in the labeled sensitivity.
STORAGE_FURNITURE_ALLOWLIST = (
    "armoire",
    "chest_of_drawers",
    "display_cabinet",
    "locker",
    "sideboard",
    "sink_cabinet",
    "wall_cabinet",
)
TABLE_ALIAS_SENSITIVITY_ALLOWLIST = ("coffee_table",)
PRIMARY_PANEL = "primary_4class_matched_overlap"
ALIAS_PANEL = "table_alias_5class_sensitivity"
PRIMARY_CATEGORIES = (
    "storage_furniture",
    "refrigerator",
    "dishwasher",
    "microwave",
)
ALIAS_CATEGORIES = (
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
)
REQUESTED_PER_CATEGORY = 6
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260812
SELECTION_SALT = "nano3d-table3-artiverse-reference-v1"
HF_REVISION = "8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248"
HF_TREE_SHA256 = "3b87f12ce58069c53784aa8df951a616a511d051dbc5939291a77381c445aee7"
HF_README_METADATA_SHA256 = "c4b23b58d0ab9fdefcd09fc2691985edcd83fe7926192cdc71fe4fc8fa0bc033"
HF_MANIFEST_METADATA_SHA256 = "701cbc373565e44b4a2425d2885a79724413267132e6127edbb8c32eb302894a"


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def validate_reference_protocol(
    protocol: dict[str, Any],
    *,
    chunk_manifest_hash: str,
    readme_hash: str,
) -> None:
    if protocol.get("protocol_id") != "artiverse_hierarchy_reference_v1":
        raise ValueError("unsupported Artiverse reference protocol id")
    if int(protocol.get("protocol_version", 0)) != 1:
        raise ValueError("unsupported Artiverse reference protocol version")
    if protocol["source"]["chunk_manifest_expected_sha256"] != chunk_manifest_hash:
        raise ValueError("source chunk manifest does not match reference protocol pin")
    if protocol["source"]["readme_expected_sha256"] != readme_hash:
        raise ValueError("Artiverse README does not match reference protocol pin")
    if protocol["source"]["huggingface_revision"] != HF_REVISION:
        raise ValueError("unsupported Hugging Face revision pin")
    if protocol["source"]["access"] != "gated" or protocol["source"]["license"] != "other":
        raise ValueError("unexpected access/license metadata in reference protocol")
    if int(protocol["selection"]["requested_per_category"]) != REQUESTED_PER_CATEGORY:
        raise ValueError("unsupported requested_per_category")
    if protocol["selection"]["rank_payload_salt"] != SELECTION_SALT:
        raise ValueError("unsupported rank payload salt")
    expected_payload_format = "\n".join(
        (
            SELECTION_SALT,
            "<target_category>",
            "<raw_category>",
            "<source>",
            "<model_id>",
        )
    )
    if protocol["selection"]["rank_payload_format"] != expected_payload_format:
        raise ValueError("rank payload format does not describe newline-delimited v1 payload")
    primary = protocol["panels"][PRIMARY_PANEL]
    alias = protocol["panels"][ALIAS_PANEL]
    if tuple(primary["target_categories"]) != PRIMARY_CATEGORIES:
        raise ValueError("primary categories differ from supported v1 contract")
    if tuple(alias["target_categories"]) != ALIAS_CATEGORIES:
        raise ValueError("alias categories differ from supported v1 contract")
    if tuple(primary["raw_category_mapping"]["storage_furniture"]) != STORAGE_FURNITURE_ALLOWLIST:
        raise ValueError("storage-furniture allowlist differs from supported v1 contract")
    if tuple(alias["raw_category_mapping"]["table"]) != TABLE_ALIAS_SENSITIVITY_ALLOWLIST:
        raise ValueError("table alias allowlist differs from supported v1 contract")
    for category in PRIMARY_CATEGORIES:
        if primary["raw_category_mapping"][category] != alias["raw_category_mapping"][category]:
            raise ValueError(f"shared raw-category mapping differs for {category}")


def identity_record(root: str, archive: str) -> dict[str, Any]:
    fields = root.split("/")
    if len(fields) != 4 or fields[0] != "data":
        raise ValueError(f"unexpected manifest root: {root!r}")
    _, raw_category, source, model_id = fields
    return {
        "raw_category": raw_category,
        "source": source,
        "model_id": model_id,
        "identity": f"{raw_category}/{source}/{model_id}",
        "manifest_root": root,
        "chunk_archive": archive,
    }


def load_identity_index(chunk_manifest: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, int]]:
    rows = []
    counts: Counter[str] = Counter()
    for chunk in chunk_manifest["chunks"]:
        for root in chunk["roots"]:
            row = identity_record(str(root), str(chunk["archive"]))
            counts[row["raw_category"]] += 1
            rows.append(row)
    if len(rows) != int(chunk_manifest["model_count"]):
        raise ValueError("chunk manifest roots do not match declared model_count")
    if len({row["identity"] for row in rows}) != len(rows):
        raise ValueError("chunk manifest contains duplicate raw identities")
    return rows, dict(sorted(counts.items()))


def rank_payload(target_category: str, identity: dict[str, Any]) -> str:
    return "\n".join(
        (
            SELECTION_SALT,
            target_category,
            str(identity["raw_category"]),
            str(identity["source"]),
            str(identity["model_id"]),
        )
    )


def freeze_one_panel(
    identity_rows: list[dict[str, Any]],
    panel: str,
    panel_protocol: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    mapping = panel_protocol["raw_category_mapping"]
    for category in panel_protocol["target_categories"]:
        raw_allowlist = set(mapping[category])
        for identity in identity_rows:
            if identity["raw_category"] not in raw_allowlist:
                continue
            payload = rank_payload(category, identity)
            candidates[category].append(
                {
                    **identity,
                    "category": category,
                    "category_mapping_kind": (
                        "nonexact_table_alias_sensitivity_only"
                        if panel == ALIAS_PANEL and category == "table"
                        else (
                            "fixed_storage_furniture_subclass_allowlist"
                            if category == "storage_furniture"
                            else "exact_raw_category"
                        )
                    ),
                    "panel": panel,
                    "rank_payload": payload,
                    "selection_hash": sha256_bytes(payload.encode("utf-8")),
                }
            )
    frozen = []
    candidate_counts = {}
    for category in panel_protocol["target_categories"]:
        ranked = sorted(
            candidates.get(category, []),
            key=lambda row: (row["selection_hash"], row["identity"]),
        )
        candidate_counts[category] = len(ranked)
        if len(ranked) < REQUESTED_PER_CATEGORY:
            raise ValueError(f"insufficient {panel} candidates for {category}: {len(ranked)}")
        for rank, row in enumerate(ranked[:REQUESTED_PER_CATEGORY], 1):
            frozen.append({**row, "selection_rank": rank})
    return frozen, candidate_counts


def build_frozen_selection(
    chunk_manifest: dict[str, Any],
    chunk_manifest_hash: str,
    reference_protocol: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    identity_rows, raw_counts = load_identity_index(chunk_manifest)
    primary, primary_counts = freeze_one_panel(
        identity_rows, PRIMARY_PANEL, reference_protocol["panels"][PRIMARY_PANEL]
    )
    alias, alias_counts = freeze_one_panel(
        identity_rows, ALIAS_PANEL, reference_protocol["panels"][ALIAS_PANEL]
    )
    exact_table_count = raw_counts.get("table", 0)
    audit = {
        "release_status": "PRE_RELEASE",
        "strict_five_category_exact_panel_feasible": exact_table_count >= REQUESTED_PER_CATEGORY,
        "strict_five_category_blockers": (
            ["exact raw category table has zero candidates; no near-category alias is admitted to the primary panel"]
            if exact_table_count == 0
            else []
        ),
        "exact_raw_category_counts": {
            category: raw_counts.get(category, 0)
            for category in ("table", "refrigerator", "dishwasher", "microwave")
        },
        "primary_panel": {
            "id": PRIMARY_PANEL,
            "categories": list(PRIMARY_CATEGORIES),
            "requested_count": len(primary),
            "candidate_counts": primary_counts,
            "interpretation": "primary four-category overlap; no table category is claimed",
        },
        "alias_sensitivity": {
            "id": ALIAS_PANEL,
            "categories": list(ALIAS_CATEGORIES),
            "requested_count": len(alias),
            "candidate_counts": alias_counts,
            "table_raw_category_allowlist": list(TABLE_ALIAS_SENSITIVITY_ALLOWLIST),
            "raw_candidate_counts": {
                category: raw_counts.get(category, 0)
                for category in TABLE_ALIAS_SENSITIVITY_ALLOWLIST
            },
            "interpretation": "nonexact category-alias sensitivity only; never pooled with primary results",
        },
        "storage_furniture_raw_category_allowlist": list(STORAGE_FURNITURE_ALLOWLIST),
        "ontology_alignment_boundary": (
            "Only raw URDF link names may enter the shared package-name alignment scorer. "
            "Artiverse articulation annotations are not PartNet semantic-hierarchy gold."
        ),
    }
    selection = {
        "protocol_id": "artiverse_pre_release_real_data_reference_selection_v1",
        "reference_protocol_id": reference_protocol["protocol_id"],
        "source_chunk_manifest_sha256": chunk_manifest_hash,
        "identity_definition": "raw_category/source/model_id",
        "selection_hash_definition": "SHA256(rank_payload UTF-8)",
        "rank_payload_format": reference_protocol["selection"]["rank_payload_format"],
        "rank_payload_salt": reference_protocol["selection"]["rank_payload_salt"],
        "ranking_rule": reference_protocol["selection"]["ranking_rule"],
        "failure_policy": "frozen failures remain in requested denominator without replacement",
        "selection_is_content_blind": True,
        "panel_definitions": {
            PRIMARY_PANEL: {
                "categories": list(PRIMARY_CATEGORIES),
                "storage_furniture_allowlist": list(STORAGE_FURNITURE_ALLOWLIST),
                "table_alias": None,
            },
            ALIAS_PANEL: {
                "categories": list(ALIAS_CATEGORIES),
                "storage_furniture_allowlist": list(STORAGE_FURNITURE_ALLOWLIST),
                "table_alias": list(TABLE_ALIAS_SENSITIVITY_ALLOWLIST),
                "status": "NONEXACT_ALIAS_SENSITIVITY_ONLY",
            },
        },
        "panels": {PRIMARY_PANEL: primary, ALIAS_PANEL: alias},
    }
    return selection, audit


def verify_source_layout(artiverse: Path, chunk_manifest: dict[str, Any]) -> dict[str, Any]:
    missing_roots = []
    non_directory_roots = []
    checked_count = 0
    for chunk in chunk_manifest["chunks"]:
        for root in chunk["roots"]:
            checked_count += 1
            candidate = artiverse / str(root)
            if not candidate.exists():
                missing_roots.append(str(root))
            elif not candidate.is_dir():
                non_directory_roots.append(str(root))
            else:
                contained(candidate)
    chunks_dir = artiverse / "dataset_chunks"
    archive_checks = {}
    for chunk in chunk_manifest["chunks"]:
        archive = chunks_dir / str(chunk["archive"])
        exists = archive.is_file()
        actual_size = archive.stat().st_size if exists else None
        expected_size = int(chunk["archive_bytes"])
        archive_checks[str(chunk["archive"])] = {
            "exists": exists,
            "expected_size_bytes": expected_size,
            "actual_size_bytes": actual_size,
            "size_matches": exists and actual_size == expected_size,
            "payload_sha256_recomputed_here": False,
            "declared_sha256": chunk.get("sha256"),
        }
    checks = {
        "checked_root_count_matches_model_count": checked_count
        == int(chunk_manifest["model_count"]),
        "all_manifest_roots_exist": not missing_roots,
        "all_manifest_roots_are_directories": not non_directory_roots,
        "all_chunk_archives_exist_with_declared_size": all(
            row["size_matches"] for row in archive_checks.values()
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "verification_method": "manifest-driven direct path existence/type checks; no data-tree traversal and no asset-content read",
        "checked_root_count": checked_count,
        "declared_model_count": int(chunk_manifest["model_count"]),
        "missing_root_count": len(missing_roots),
        "missing_roots": missing_roots,
        "non_directory_root_count": len(non_directory_roots),
        "non_directory_roots": non_directory_roots,
        "archive_checks": archive_checks,
    }


def read_hf_metadata(path: Path, artiverse: Path) -> dict[str, Any]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if len(lines) < 2:
        raise ValueError(f"invalid Hugging Face metadata: {path}")
    return {
        "path": path.relative_to(artiverse).as_posix(),
        "revision": lines[0],
        "blob_or_lfs_oid": lines[1],
        "sha256": sha256_file(path),
    }


def verify_hf_revision_evidence(
    artiverse: Path,
    chunk_manifest: dict[str, Any],
) -> dict[str, Any]:
    cache = artiverse / ".cache/huggingface"
    tree_path = cache / f"trees/{HF_REVISION}.json"
    readme_metadata_path = cache / "download/README.md.metadata"
    manifest_metadata_path = cache / "download/dataset_chunks/manifest.json.metadata"
    tree = json.loads(tree_path.read_text(encoding="utf-8"))
    readme_metadata = read_hf_metadata(readme_metadata_path, artiverse)
    manifest_metadata = read_hf_metadata(manifest_metadata_path, artiverse)
    files = tree["files"]
    chunk_checks = {}
    for chunk in chunk_manifest["chunks"]:
        archive = str(chunk["archive"])
        relative = f"dataset_chunks/{archive}"
        metadata_path = cache / f"download/{relative}.metadata"
        metadata = read_hf_metadata(metadata_path, artiverse)
        chunk_checks[archive] = {
            "metadata": metadata,
            "tree_lfs_sha256": files[relative].get("lfs_sha256"),
            "manifest_declared_sha256": chunk.get("sha256"),
            "revision_matches": metadata["revision"] == HF_REVISION,
            "metadata_oid_matches_tree_lfs": metadata["blob_or_lfs_oid"]
            == files[relative].get("lfs_sha256"),
            "tree_lfs_matches_manifest": files[relative].get("lfs_sha256")
            == chunk.get("sha256"),
        }
    checks = {
        "tree_evidence_sha256_matches_pin": sha256_file(tree_path) == HF_TREE_SHA256,
        "readme_metadata_sha256_matches_pin": readme_metadata["sha256"]
        == HF_README_METADATA_SHA256,
        "manifest_metadata_sha256_matches_pin": manifest_metadata["sha256"]
        == HF_MANIFEST_METADATA_SHA256,
        "readme_revision_matches": readme_metadata["revision"] == HF_REVISION,
        "manifest_revision_matches": manifest_metadata["revision"] == HF_REVISION,
        "readme_blob_matches_tree": readme_metadata["blob_or_lfs_oid"]
        == files["README.md"]["blob_id"],
        "manifest_blob_matches_tree": manifest_metadata["blob_or_lfs_oid"]
        == files["dataset_chunks/manifest.json"]["blob_id"],
        "all_chunk_lfs_evidence_matches": all(
            row["revision_matches"]
            and row["metadata_oid_matches_tree_lfs"]
            and row["tree_lfs_matches_manifest"]
            for row in chunk_checks.values()
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "huggingface_revision": HF_REVISION,
        "access": "gated",
        "license": "other",
        "release_status": "PRE_RELEASE",
        "tree_evidence": {
            "path": str(tree_path.relative_to(artiverse)),
            "sha256": sha256_file(tree_path),
        },
        "readme_metadata": readme_metadata,
        "manifest_metadata": manifest_metadata,
        "chunk_checks": chunk_checks,
    }


def validate_frozen_selection(
    selection: dict[str, Any],
    chunk_manifest_hash: str,
    reference_protocol: dict[str, Any],
) -> None:
    if selection.get("source_chunk_manifest_sha256") != chunk_manifest_hash:
        raise ValueError("frozen selection source manifest hash mismatch")
    expected = {PRIMARY_PANEL: PRIMARY_CATEGORIES, ALIAS_PANEL: ALIAS_CATEGORIES}
    for panel, categories in expected.items():
        rows = selection.get("panels", {}).get(panel, [])
        if Counter(row["category"] for row in rows) != Counter(
            {category: REQUESTED_PER_CATEGORY for category in categories}
        ):
            raise ValueError(f"invalid frozen category counts for {panel}")
        for row in rows:
            identity = f"{row['raw_category']}/{row['source']}/{row['model_id']}"
            if row["identity"] != identity:
                raise ValueError(f"frozen identity mismatch: {row}")
            payload = rank_payload(row["category"], row)
            if row["rank_payload"] != payload:
                raise ValueError(f"frozen rank payload mismatch: {identity}")
            if row["selection_hash"] != sha256_bytes(payload.encode("utf-8")):
                raise ValueError(f"frozen selection hash mismatch: {identity}")
    primary = selection["panels"][PRIMARY_PANEL]
    alias = selection["panels"][ALIAS_PANEL]
    for category in PRIMARY_CATEGORIES:
        primary_rows = [row["identity"] for row in primary if row["category"] == category]
        alias_rows = [row["identity"] for row in alias if row["category"] == category]
        if primary_rows != alias_rows:
            raise ValueError(f"shared panel selection mismatch for {category}")
    if reference_protocol["selection"]["rank_payload_salt"] != SELECTION_SALT:
        raise ValueError("unsupported reference protocol selection salt")


def select_direct_urdf(model_root: Path) -> tuple[Path, dict[str, Any]]:
    urdf_dir = model_root / "urdf_w_collider"
    if not urdf_dir.is_dir():
        raise FileNotFoundError("missing urdf_w_collider directory")
    urdf_candidates = sorted(
        path for path in urdf_dir.iterdir()
        if path.is_file() and path.suffix.lower() == ".urdf"
    )
    if not urdf_candidates:
        raise FileNotFoundError("no direct URDF in urdf_w_collider")
    # Deterministic package convention, independent of parse success.
    preferred_urdf = [path for path in urdf_candidates if path.name in {"mobility.urdf", "model.urdf"}]
    urdf = (preferred_urdf or urdf_candidates)[0]

    return urdf, {
        "urdf_candidate_names": [path.name for path in urdf_candidates],
        "urdf_selection_rule": "preferred exact mobility.urdf/model.urdf, else lexicographically first direct URDF",
    }


def select_direct_articulation(model_root: Path, model_id: str) -> tuple[Path, dict[str, Any]]:
    expected_annotation = model_root / f"{model_id}.articulations.json"
    annotation_candidates = sorted(
        path for path in model_root.iterdir()
        if path.is_file() and path.name.endswith(".articulations.json")
    )
    if expected_annotation.is_file():
        annotation = expected_annotation
    elif annotation_candidates:
        annotation = annotation_candidates[0]
    else:
        raise FileNotFoundError("no direct *.articulations.json")
    return annotation, {
        "articulation_candidate_names": [path.name for path in annotation_candidates],
        "articulation_selection_rule": "expected {model_id}.articulations.json, else lexicographically first direct suffix match",
    }


def articulation_summary(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, list):
        records = payload
        container = "list"
        record_field = None
    elif isinstance(payload, dict):
        container = "dict"
        list_fields = [(key, value) for key, value in payload.items() if isinstance(value, list)]
        preferred = [item for item in list_fields if item[0] in {"articulations", "joints"}]
        record_field, records = (preferred or list_fields)[0] if (preferred or list_fields) else (None, [])
    else:
        raise ValueError(f"unexpected annotation root: {type(payload).__name__}")
    joint_types: Counter[str] = Counter()
    for record in records:
        if isinstance(record, dict):
            value = record.get("joint_type", record.get("type", record.get("motion_type")))
            joint_types[str(value) if value is not None else "unspecified"] += 1
        else:
            joint_types["non_object_record"] += 1
    return {
        "annotation_root_type": container,
        "annotation_record_field": record_field,
        "annotation_record_count": len(records),
        "annotation_joint_type_counts": dict(sorted(joint_types.items())),
        "interpretation": "package articulation metadata only; not PartNet ontology ground truth",
    }


def aggregate_panel_structure(records: list[dict[str, Any]], categories: tuple[str, ...]) -> dict[str, Any]:
    per_category = {}
    for category in categories:
        rows = [row for row in records if row["category"] == category]
        evaluated = [row for row in rows if row.get("evaluation_complete")]
        valid = [row for row in evaluated if row.get("valid_tree")]
        joint_types = Counter(
            {
                joint_type: sum(
                    int(row.get("joint_type_counts", {}).get(joint_type, 0))
                    for row in evaluated
                )
                for joint_type in sorted(
                    {
                        joint_type
                        for row in evaluated
                        for joint_type in row.get("joint_type_counts", {})
                    }
                )
            }
        )
        unsupported = {
            joint_type: count
            for joint_type, count in joint_types.items()
            if joint_type not in MOVABLE_TYPES | {"fixed"}
        }
        all_nonfixed_count = sum(
            count for joint_type, count in joint_types.items() if joint_type != "fixed"
        )
        valid_nonfixed_count = sum(
            int(row.get("edge_count", 0)) - int(row.get("fixed_edge_count", 0))
            for row in valid
        )
        per_category[category] = {
            "requested_count": len(rows),
            "available_count": sum(bool(row.get("available")) for row in rows),
            "evaluation_complete_count": len(evaluated),
            "metrics": aggregate_structure(evaluated, requested_count=len(rows)),
            "topology_consistency": topology_consistency(valid),
            "joint_type_counts_evaluated": dict(joint_types),
            "unsupported_or_other_joint_type_counts_evaluated": unsupported,
            "unsupported_or_other_joint_count_evaluated": sum(unsupported.values()),
            "all_nonfixed_joint_count_evaluated": all_nonfixed_count,
            "all_nonfixed_joint_count_valid": valid_nonfixed_count,
            "all_nonfixed_joint_count_mean_valid": (
                valid_nonfixed_count / len(valid) if valid else None
            ),
        }
        available_count = per_category[category]["available_count"]
        metrics = per_category[category]["metrics"]
        per_category[category]["rates"] = {
            "available_requested": available_count / len(rows),
            "parsed_requested": len(evaluated) / len(rows),
            "valid_available": (
                metrics["valid_tree_count"] / available_count
                if available_count
                else None
            ),
            "valid_requested": metrics["valid_tree_count"] / len(rows),
        }
    evaluated = [row for row in records if row.get("evaluation_complete")]
    valid = [row for row in evaluated if row.get("valid_tree")]
    overall = aggregate_structure(evaluated, requested_count=len(records))
    joint_types = Counter(
        joint_type
        for row in evaluated
        for joint_type, count in row.get("joint_type_counts", {}).items()
        for _ in range(int(count))
    )
    unsupported = {
        joint_type: count
        for joint_type, count in sorted(joint_types.items())
        if joint_type not in MOVABLE_TYPES | {"fixed"}
    }
    all_nonfixed_count = sum(
        count for joint_type, count in joint_types.items() if joint_type != "fixed"
    )
    valid_nonfixed_count = sum(
        int(row.get("edge_count", 0)) - int(row.get("fixed_edge_count", 0))
        for row in valid
    )
    overall.update(
        {
            "available_count": sum(bool(row.get("available")) for row in records),
            "evaluation_complete_count": len(evaluated),
            "unavailable_count": sum(not bool(row.get("available")) for row in records),
            "parse_failure_count": sum(
                bool(row.get("available")) and not bool(row.get("evaluation_complete"))
                for row in records
            ),
            "joint_type_counts_evaluated": dict(sorted(joint_types.items())),
            "unsupported_or_other_joint_type_counts_evaluated": unsupported,
            "unsupported_or_other_joint_count_evaluated": sum(unsupported.values()),
            "all_nonfixed_joint_count_evaluated": all_nonfixed_count,
            "all_nonfixed_joint_count_valid": valid_nonfixed_count,
            "all_nonfixed_joint_count_mean_valid": (
                valid_nonfixed_count / len(valid) if valid else None
            ),
        }
    )
    overall["rates"] = {
        "available_requested": overall["available_count"] / len(records),
        "parsed_requested": overall["evaluation_complete_count"] / len(records),
        "valid_available": (
            overall["valid_tree_count"] / overall["available_count"]
            if overall["available_count"]
            else None
        ),
        "valid_requested": overall["valid_tree_count"] / len(records),
    }
    fields = ("unique_signature_rate", "mode_rate", "pairwise_exact_rate", "normalized_entropy")
    category_macro = {}
    for field in fields:
        values = [
            float(per_category[category]["topology_consistency"][field])
            for category in categories
            if per_category[category]["topology_consistency"][field] is not None
        ]
        category_macro[field] = sum(values) / len(values) if values else None
    category_macro_structure = {}
    for rate in ("available_requested", "parsed_requested", "valid_available", "valid_requested"):
        values = [
            float(per_category[category]["rates"][rate])
            for category in categories
            if per_category[category]["rates"][rate] is not None
        ]
        category_macro_structure[rate] = sum(values) / len(values) if values else None
    for metric in ("node_count_mean", "semantic_depth_mean", "movable_edge_count_mean"):
        values = [
            float(per_category[category]["metrics"][metric])
            for category in categories
            if per_category[category]["metrics"]["valid_tree_count"] > 0
            and per_category[category]["metrics"][metric] is not None
        ]
        category_macro_structure[f"{metric}_valid_only"] = (
            sum(values) / len(values) if values else None
        )
    nonfixed_values = [
        float(per_category[category]["all_nonfixed_joint_count_mean_valid"])
        for category in categories
        if per_category[category]["all_nonfixed_joint_count_mean_valid"] is not None
    ]
    category_macro_structure["all_nonfixed_joint_count_mean_valid"] = (
        sum(nonfixed_values) / len(nonfixed_values) if nonfixed_values else None
    )
    return {
        "overall": overall,
        "per_category": per_category,
        "category_macro_topology_consistency": category_macro,
        "category_macro_structure": category_macro_structure,
        "pooled_topology_consistency_diagnostic": topology_consistency(valid),
        "topology_primary_weighting": f"simple mean over {len(categories)} per-category valid-asset statistics",
        "structure_conditioning": "node/depth/movable-joint means are conditional on valid trees only",
    }


def coverage_weighted_f1(row: dict[str, Any]) -> float:
    return float(row.get("parent_child_edge_f1") or 0.0) * float(
        row.get("semantic_role_coverage") or 0.0
    )


def quantile(values: list[float], probability: float) -> float:
    position = (len(values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def bootstrap_alignment(records: list[dict[str, Any]], categories: tuple[str, ...]) -> dict[str, Any]:
    grouped = {category: [row for row in records if row["category"] == category] for category in categories}
    if any(len(rows) != REQUESTED_PER_CATEGORY for rows in grouped.values()):
        raise ValueError("bootstrap requires six frozen records per category")
    estimate = sum(coverage_weighted_f1(row) for row in records) / len(records)
    rng = random.Random(BOOTSTRAP_SEED)
    samples = []
    for _ in range(BOOTSTRAP_REPLICATES):
        sampled = [rng.choice(grouped[category]) for category in categories for _ in range(REQUESTED_PER_CATEGORY)]
        samples.append(sum(coverage_weighted_f1(row) for row in sampled) / len(sampled))
    samples.sort()
    return {
        "metric": "coverage_weighted_induced_edge_f1_requested_macro",
        "estimate": estimate,
        "ci95_percentile": [quantile(samples, 0.025), quantile(samples, 0.975)],
        "design": f"category-stratified bootstrap; six draws with replacement within each of {len(categories)} categories",
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "ranking_or_pairwise_difference": False,
    }


def frozen_composition(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "raw_category_counts": dict(
            sorted(Counter(row["raw_category"] for row in rows).items())
        ),
        "source_counts": dict(sorted(Counter(row["source"] for row in rows).items())),
        "target_by_raw_category_counts": {
            category: dict(
                sorted(
                    Counter(
                        row["raw_category"]
                        for row in rows
                        if row["category"] == category
                    ).items()
                )
            )
            for category in sorted({row["category"] for row in rows})
        },
        "target_by_source_counts": {
            category: dict(
                sorted(
                    Counter(
                        row["source"]
                        for row in rows
                        if row["category"] == category
                    ).items()
                )
            )
            for category in sorted({row["category"] for row in rows})
        },
    }


def evaluate_physical_assets(
    artiverse: Path,
    output: Path,
    selection: dict[str, Any],
) -> tuple[dict[str, dict[str, Any]], list[dict[str, Any]]]:
    union = {}
    for rows in selection["panels"].values():
        for row in rows:
            union[row["identity"]] = row
    extracted_root = output / "selected_metadata"
    extracted_root.mkdir(parents=True, exist_ok=True)
    physical = {}
    annotations = []
    for identity in sorted(union):
        selected = union[identity]
        model_root_path = artiverse / selected["manifest_root"]
        record: dict[str, Any] = {
            "identity": identity,
            "raw_category": selected["raw_category"],
            "source": selected["source"],
            "model_id": selected["model_id"],
            "manifest_root": selected["manifest_root"],
            "chunk_archive": selected["chunk_archive"],
            "model_root": str(model_root_path),
            "available": False,
            "urdf_available": False,
            "articulation_available": False,
            "availability_error": None,
            "urdf_availability_error": None,
            "articulation_availability_error": None,
            "selection_used_file_content": False,
            "failure_replaced": False,
        }
        try:
            model_root = contained(model_root_path)
            if not model_root.is_dir():
                raise FileNotFoundError(f"frozen model directory unavailable: {model_root}")
            destination = extracted_root / selected["selection_hash"]
            destination.mkdir(parents=True, exist_ok=True)
            record["files"] = {}
            record["package_file_resolution"] = {}
            try:
                urdf, resolution = select_direct_urdf(model_root)
                urdf_copy = destination / "model.urdf"
                shutil.copy2(urdf, urdf_copy)
                record["files"]["urdf"] = {
                    "source_path": str(urdf),
                    "path": urdf_copy.relative_to(output).as_posix(),
                    "sha256": sha256_file(urdf_copy),
                    "size_bytes": urdf_copy.stat().st_size,
                }
                record["package_file_resolution"].update(resolution)
                record["urdf_available"] = True
                record["available"] = True
            except Exception as exc:
                record["urdf_availability_error"] = f"{type(exc).__name__}: {exc}"
                record["availability_error"] = record["urdf_availability_error"]
            try:
                annotation, resolution = select_direct_articulation(model_root, selected["model_id"])
                annotation_copy = destination / "articulations.json"
                shutil.copy2(annotation, annotation_copy)
                record["files"]["articulations"] = {
                    "source_path": str(annotation),
                    "path": annotation_copy.relative_to(output).as_posix(),
                    "sha256": sha256_file(annotation_copy),
                    "size_bytes": annotation_copy.stat().st_size,
                }
                record["package_file_resolution"].update(resolution)
                record["articulation_available"] = True
            except Exception as exc:
                record["articulation_availability_error"] = f"{type(exc).__name__}: {exc}"
        except Exception as exc:
            record["availability_error"] = f"{type(exc).__name__}: {exc}"
            record["urdf_availability_error"] = record["availability_error"]
            record["articulation_availability_error"] = record["availability_error"]
        physical[identity] = record

        annotation_record = {
            "identity": identity,
            "available": record["articulation_available"],
            "evaluation_complete": False,
            "evaluation_error": record["articulation_availability_error"],
        }
        if record["articulation_available"]:
            try:
                annotation_record.update(
                    articulation_summary(output / record["files"]["articulations"]["path"])
                )
                annotation_record["evaluation_complete"] = True
                annotation_record["evaluation_error"] = None
            except Exception as exc:
                annotation_record["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        annotations.append(annotation_record)
    return physical, annotations


def evaluate_panel(
    panel: str,
    frozen: list[dict[str, Any]],
    physical: dict[str, dict[str, Any]],
    output: Path,
    protocol: dict[str, Any],
    categories: tuple[str, ...],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    manifest_rows = []
    structure_rows = []
    alignment_rows = []
    for selected in frozen:
        package = physical[selected["identity"]]
        manifest_row = {
            "reference": "Artiverse (pre-release real-data reference)",
            **selected,
            **{key: value for key, value in package.items() if key not in {"identity", "raw_category", "source", "model_id", "manifest_root", "chunk_archive"}},
        }
        manifest_rows.append(manifest_row)

        structure_row = dict(manifest_row)
        structure_row["evaluation_complete"] = False
        if manifest_row["available"]:
            try:
                structure_row.update(analyze_urdf(output / manifest_row["files"]["urdf"]["path"]))
                structure_row["evaluation_complete"] = True
                structure_row["evaluation_error"] = None
            except Exception as exc:
                structure_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        structure_rows.append(structure_row)

        alignment_row = dict(manifest_row)
        alignment_row["evaluation_complete"] = False
        alignment_row["alignment_input"] = "raw package URDF link names only"
        alignment_row["artiverse_articulation_annotation_used_for_roles"] = False
        alignment_row["claim_boundary"] = "package-name semantic recoverability and ontology-alignment sensitivity; not kinematic correctness"
        if manifest_row["available"]:
            try:
                alignment_row.update(
                    evaluate_urdf(
                        output / manifest_row["files"]["urdf"]["path"],
                        selected["category"],
                        protocol,
                    )
                )
                alignment_row["evaluation_complete"] = True
                alignment_row["evaluation_error"] = None
            except Exception as exc:
                alignment_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        alignment_rows.append(alignment_row)

    panel_summary = {
        "panel": panel,
        "categories": list(categories),
        "requested_count": len(frozen),
        "requested_per_category": REQUESTED_PER_CATEGORY,
        "structure": aggregate_panel_structure(structure_rows, categories),
        "urdf_name_only_ontology_alignment_sensitivity": aggregate_alignment(alignment_rows),
        "urdf_name_only_alignment_bootstrap": bootstrap_alignment(alignment_rows, categories),
        "artiverse_articulation_annotations_used_for_alignment": False,
    }
    return manifest_rows, structure_rows, alignment_rows, panel_summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artiverse-root", type=Path, default=DEFAULT_ARTIVERSE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument(
        "--reference-protocol", type=Path, default=DEFAULT_REFERENCE_PROTOCOL
    )
    parser.add_argument("--selection", type=Path)
    parser.add_argument("--freeze-only", action="store_true")
    args = parser.parse_args()

    artiverse = contained(args.artiverse_root)
    output = contained(args.output, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    chunk_manifest_path = contained(artiverse / "dataset_chunks/manifest.json")
    readme_path = contained(artiverse / "README.md")
    protocol_path = contained(args.protocol)
    reference_protocol_path = contained(args.reference_protocol)
    chunk_manifest_hash = sha256_file(chunk_manifest_path)
    readme_hash = sha256_file(readme_path)
    chunk_manifest = json.loads(chunk_manifest_path.read_text(encoding="utf-8"))
    reference_protocol = json.loads(
        reference_protocol_path.read_text(encoding="utf-8")
    )
    validate_reference_protocol(
        reference_protocol,
        chunk_manifest_hash=chunk_manifest_hash,
        readme_hash=readme_hash,
    )
    source_layout = verify_source_layout(artiverse, chunk_manifest)
    source_layout_path = output / "manifest_root_verification.json"
    write_json(source_layout_path, source_layout)
    if not source_layout["passed"]:
        raise RuntimeError(
            "Artiverse source-layout verification failed before selection freeze: "
            f"{source_layout['checks']}"
        )
    hf_revision_evidence = verify_hf_revision_evidence(artiverse, chunk_manifest)
    hf_revision_path = output / "hf_revision_verification.json"
    write_json(hf_revision_path, hf_revision_evidence)
    if not hf_revision_evidence["passed"]:
        raise RuntimeError(
            "Artiverse Hugging Face revision verification failed before selection freeze: "
            f"{hf_revision_evidence['checks']}"
        )
    protocol_snapshot_path = output / "reference_protocol_snapshot.json"
    write_json(protocol_snapshot_path, reference_protocol)
    derived_selection, audit = build_frozen_selection(
        chunk_manifest, chunk_manifest_hash, reference_protocol
    )

    supplied_selection_path = contained(args.selection) if args.selection else None
    local_selection_path = output / "frozen_selection.json"
    selection_input_path = supplied_selection_path or local_selection_path
    if selection_input_path.exists():
        selection = json.loads(selection_input_path.read_text(encoding="utf-8"))
        if selection != derived_selection:
            raise ValueError("existing frozen selection differs from current identity-only derivation")
    else:
        selection = derived_selection
    if local_selection_path.exists():
        local_selection = json.loads(local_selection_path.read_text(encoding="utf-8"))
        if local_selection != selection:
            raise ValueError("local frozen selection differs from supplied/derived selection")
    else:
        write_json(local_selection_path, selection)
    validate_frozen_selection(selection, chunk_manifest_hash, reference_protocol)
    write_json(output / "category_feasibility_audit.json", audit)
    if args.freeze_only:
        print(
            json.dumps(
                {
                    "selection": str(local_selection_path),
                    "selection_sha256": sha256_file(local_selection_path),
                    "reference_protocol_snapshot_sha256": sha256_file(
                        protocol_snapshot_path
                    ),
                    "manifest_root_verification_sha256": sha256_file(
                        source_layout_path
                    ),
                    "hf_revision_verification_sha256": sha256_file(
                        hf_revision_path
                    ),
                    "primary_requested": len(selection["panels"][PRIMARY_PANEL]),
                    "alias_sensitivity_requested": len(selection["panels"][ALIAS_PANEL]),
                    "unique_physical_assets": len(
                        {row["identity"] for rows in selection["panels"].values() for row in rows}
                    ),
                    "strict_five_category_exact_panel_feasible": audit["strict_five_category_exact_panel_feasible"],
                },
                indent=2,
            )
        )
        return 0

    protocol = load_protocol(protocol_path)
    physical, annotation_records = evaluate_physical_assets(artiverse, output, selection)
    panel_outputs = {}
    artifact_hashes = {}
    for panel, categories in ((PRIMARY_PANEL, PRIMARY_CATEGORIES), (ALIAS_PANEL, ALIAS_CATEGORIES)):
        manifests, structures, alignments, panel_summary = evaluate_panel(
            panel,
            selection["panels"][panel],
            physical,
            output,
            protocol,
            categories,
        )
        manifest_name = "manifest.jsonl" if panel == PRIMARY_PANEL else "alias_sensitivity_manifest.jsonl"
        paths = {
            "manifest": output / manifest_name,
            "structure_records": output / f"{panel}_structure_records.jsonl",
            "alignment_records": output / f"{panel}_urdf_name_only_alignment_records.jsonl",
        }
        write_jsonl(paths["manifest"], manifests)
        write_jsonl(paths["structure_records"], structures)
        write_jsonl(paths["alignment_records"], alignments)
        panel_outputs[panel] = panel_summary
        artifact_hashes[panel] = {
            f"{key}_sha256": sha256_file(path) for key, path in paths.items()
        }

    annotation_path = output / "articulation_metadata_records.jsonl"
    write_jsonl(annotation_path, annotation_records)
    provenance = {
        "dataset": "Artiverse",
        "release_status": "PRE_RELEASE",
        "release_warning": "README states that this subset is under development and continuously verified/improved",
        "role": "curated real-data reference; not a generation method; excluded from generated-method rankings",
        "source": {
            "readme_path": str(readme_path),
            "readme_sha256": readme_hash,
            "chunk_manifest_path": str(chunk_manifest_path),
            "chunk_manifest_sha256": chunk_manifest_hash,
            "chunk_manifest_created_utc": chunk_manifest.get("created_utc"),
            "chunk_manifest_model_count": chunk_manifest.get("model_count"),
            "chunk_manifest_declared_chunk_hashes": {
                row["archive"]: row.get("sha256") for row in chunk_manifest["chunks"]
            },
            "manifest_root_verification_path": "manifest_root_verification.json",
            "manifest_root_verification_sha256": sha256_file(source_layout_path),
            "all_manifest_roots_verified": source_layout["passed"],
            "hf_revision_verification_path": "hf_revision_verification.json",
            "hf_revision_verification_sha256": sha256_file(hf_revision_path),
            "huggingface_revision_verified": hf_revision_evidence["passed"],
            "huggingface_revision": hf_revision_evidence["huggingface_revision"],
            "access": hf_revision_evidence["access"],
            "license": hf_revision_evidence["license"],
            "chunk_payloads_rehashed_by_runner": False,
            "reason_not_rehashed": "runner consumes the signed-off manifest and only reads the 30 frozen direct model paths",
        },
        "selection": {
            "path": "frozen_selection.json",
            "sha256": sha256_file(local_selection_path),
            "identity_only": True,
            "content_blind": True,
            "failure_policy": "no replacement",
            "unique_physical_asset_count": len(physical),
            "primary_composition": frozen_composition(
                selection["panels"][PRIMARY_PANEL]
            ),
            "alias_sensitivity_composition": frozen_composition(
                selection["panels"][ALIAS_PANEL]
            ),
        },
        "protocol": {"path": str(protocol_path), "sha256": sha256_file(protocol_path)},
        "reference_protocol": {
            "path": "reference_protocol_snapshot.json",
            "sha256": sha256_file(protocol_snapshot_path),
            "protocol_id": reference_protocol["protocol_id"],
        },
        "ontology_alignment_boundary": (
            "Shared scorer receives raw URDF link names only. Artiverse articulation annotation is never supplied as role gold; "
            "the resulting metric is package-name recoverability and induced PartNet-ontology alignment sensitivity only."
        ),
        "paper_ready": False,
        "paper_ready_blockers": [
            "Artiverse subset is labeled pre-release and undergoing cleanup",
            "the exact five-category Table 3 panel is infeasible because exact table is absent",
            "the five-category result requires a disclosed nonexact coffee_table->table alias",
        ],
    }
    provenance_path = output / "provenance.json"
    write_json(provenance_path, provenance)

    summary = {
        "protocol_id": "artiverse_pre_release_real_data_reference_v1",
        "display_name": "Artiverse (pre-release real-data reference)",
        "role": provenance["role"],
        "strict_five_category_exact_panel_feasible": audit["strict_five_category_exact_panel_feasible"],
        "frozen_composition": {
            "primary": frozen_composition(selection["panels"][PRIMARY_PANEL]),
            "table_alias_sensitivity": frozen_composition(
                selection["panels"][ALIAS_PANEL]
            ),
        },
        "primary": panel_outputs[PRIMARY_PANEL],
        "table_alias_sensitivity": panel_outputs[ALIAS_PANEL],
        "articulation_metadata": {
            "unique_physical_asset_count": len(annotation_records),
            "available_count": sum(bool(row["available"]) for row in annotation_records),
            "parse_complete_count": sum(bool(row["evaluation_complete"]) for row in annotation_records),
            "record_count_total": sum(int(row.get("annotation_record_count", 0)) for row in annotation_records),
            "used_as_partnet_gold": False,
        },
        "paper_ready": False,
        "hashes": {
            "frozen_selection_sha256": sha256_file(local_selection_path),
            "category_feasibility_audit_sha256": sha256_file(output / "category_feasibility_audit.json"),
            "manifest_root_verification_sha256": sha256_file(source_layout_path),
            "hf_revision_verification_sha256": sha256_file(hf_revision_path),
            "reference_protocol_snapshot_sha256": sha256_file(protocol_snapshot_path),
            "articulation_metadata_records_sha256": sha256_file(annotation_path),
            "provenance_sha256": sha256_file(provenance_path),
            "runner_sha256": sha256_file(Path(__file__)),
            "panel_artifacts": artifact_hashes,
        },
    }
    summary_path = output / "summary.json"
    write_json(summary_path, summary)

    primary = summary["primary"]
    alias = summary["table_alias_sensitivity"]
    p_structure = primary["structure"]["overall"]
    a_structure = alias["structure"]["overall"]
    p_align = primary["urdf_name_only_ontology_alignment_sensitivity"]
    a_align = alias["urdf_name_only_ontology_alignment_sensitivity"]
    p_boot = primary["urdf_name_only_alignment_bootstrap"]
    a_boot = alias["urdf_name_only_alignment_bootstrap"]
    per_category_lines = []
    for category in PRIMARY_CATEGORIES:
        row = primary["structure"]["per_category"][category]
        metrics = row["metrics"]
        per_category_lines.append(
            f"| {category} | {row['available_count']}/6 | "
            f"{row['evaluation_complete_count']}/6 | "
            f"{metrics['valid_tree_count']}/{row['available_count']} | "
            f"{metrics['valid_tree_count']}/6 | "
            f"{metrics['node_count_mean'] or 0:.3f} | "
            f"{metrics['semantic_depth_mean'] or 0:.3f} | "
            f"{metrics['movable_edge_count_mean'] or 0:.3f} |"
        )
    p_macro = primary["structure"]["category_macro_structure"]
    a_macro = alias["structure"]["category_macro_structure"]
    report = [
        "# Artiverse pre-release real-data reference",
        "",
        "Artiverse is a curated real-data reference, not a generation method, and is excluded from generated-method rankings.",
        "",
        "## Category feasibility",
        "",
        "The strict five-category exact panel is infeasible because this manifest has no exact `table` category. The primary result therefore uses the four-category matched overlap (N=24): three exact appliance categories plus the pre-specified storage-furniture subclass crosswalk. A separately labeled sensitivity maps `coffee_table` to `table` (N=30); it is not an exact-category result and is never pooled with the primary panel.",
        "",
        "## Frozen package structure",
        "",
        "| Panel | Available / requested | Parsed / requested | Valid / available | Valid / requested | Nodes / valid | Depth / valid | Movable joints / valid |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        f"| Primary four-class overlap | {p_structure['available_count']}/24 | {p_structure['evaluation_complete_count']}/24 | {p_structure['valid_tree_count']}/{p_structure['available_count']} | {p_structure['valid_tree_count']}/24 | {p_structure['node_count_mean'] or 0:.3f} | {p_structure['semantic_depth_mean'] or 0:.3f} | {p_structure['movable_edge_count_mean'] or 0:.3f} |",
        f"| Five-class table-alias sensitivity | {a_structure['available_count']}/30 | {a_structure['evaluation_complete_count']}/30 | {a_structure['valid_tree_count']}/{a_structure['available_count']} | {a_structure['valid_tree_count']}/30 | {a_structure['node_count_mean'] or 0:.3f} | {a_structure['semantic_depth_mean'] or 0:.3f} | {a_structure['movable_edge_count_mean'] or 0:.3f} |",
        "",
        "Nodes, depth, and movable joints are computed on valid trees only.",
        "",
        "### Primary panel by category",
        "",
        "| Category | Available / 6 | Parsed / 6 | Valid / available | Valid / 6 | Nodes / valid | Depth / valid | Movable joints / valid |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        *per_category_lines,
        "",
        "| Panel | Category-macro available | Category-macro parsed | Category-macro valid / available | Category-macro valid / requested | Category-macro nodes / valid | Category-macro depth / valid | Category-macro movable joints / valid |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
        f"| Primary | {100*p_macro['available_requested']:.1f}% | {100*p_macro['parsed_requested']:.1f}% | {100*(p_macro['valid_available'] or 0):.1f}% | {100*p_macro['valid_requested']:.1f}% | {p_macro['node_count_mean_valid_only'] or 0:.3f} | {p_macro['semantic_depth_mean_valid_only'] or 0:.3f} | {p_macro['movable_edge_count_mean_valid_only'] or 0:.3f} |",
        f"| Alias sensitivity | {100*a_macro['available_requested']:.1f}% | {100*a_macro['parsed_requested']:.1f}% | {100*(a_macro['valid_available'] or 0):.1f}% | {100*a_macro['valid_requested']:.1f}% | {a_macro['node_count_mean_valid_only'] or 0:.3f} | {a_macro['semantic_depth_mean_valid_only'] or 0:.3f} | {a_macro['movable_edge_count_mean_valid_only'] or 0:.3f} |",
        "",
        f"Primary evaluated joint types: `{json.dumps(p_structure['joint_type_counts_evaluated'], sort_keys=True)}`; unsupported/other: `{json.dumps(p_structure['unsupported_or_other_joint_type_counts_evaluated'], sort_keys=True)}` (N={p_structure['unsupported_or_other_joint_count_evaluated']}).",
        f"Alias-sensitivity evaluated joint types: `{json.dumps(a_structure['joint_type_counts_evaluated'], sort_keys=True)}`; unsupported/other: `{json.dumps(a_structure['unsupported_or_other_joint_type_counts_evaluated'], sort_keys=True)}` (N={a_structure['unsupported_or_other_joint_count_evaluated']}).",
        "",
        "## URDF-name-only ontology-alignment sensitivity",
        "",
        "| Panel | Role coverage / requested | Scorable / requested | Coverage-weighted induced Edge F1 (95% CI) | Parent alignment / requested |",
        "|---|---:|---:|---:|---:|",
        f"| Primary four-class matched overlap | {100*p_align['semantic_role_coverage_requested_macro']:.1f}% | {100*p_align['scorable_asset_coverage_requested']:.1f}% | {100*p_align['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% [{100*p_boot['ci95_percentile'][0]:.1f}, {100*p_boot['ci95_percentile'][1]:.1f}] | {100*p_align['semantic_nesting_accuracy_requested_macro']:.1f}% |",
        f"| Five-class table-alias sensitivity | {100*a_align['semantic_role_coverage_requested_macro']:.1f}% | {100*a_align['scorable_asset_coverage_requested']:.1f}% | {100*a_align['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% [{100*a_boot['ci95_percentile'][0]:.1f}, {100*a_boot['ci95_percentile'][1]:.1f}] | {100*a_align['semantic_nesting_accuracy_requested_macro']:.1f}% |",
        "",
        "The shared scorer sees raw package URDF link names only. Artiverse articulation JSON is audited as package metadata but is not used as PartNet role ground truth. These values measure semantic recoverability and induced ontology alignment, not instance-level kinematic correctness.",
        "",
        "## Status",
        "",
        "`PRE_RELEASE`; `PAPER_READY=false`. Use only as a clearly labeled reference/sensitivity until the release is finalized.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    expected_primary = Counter({category: 6 for category in PRIMARY_CATEGORIES})
    expected_alias = Counter({category: 6 for category in ALIAS_CATEGORIES})
    primary_manifest = selection["panels"][PRIMARY_PANEL]
    alias_manifest = selection["panels"][ALIAS_PANEL]
    verification = {
        "passed": True,
        "checks": {
            "primary_has_24_frozen_rows": len(primary_manifest) == 24,
            "primary_has_six_per_category": Counter(row["category"] for row in primary_manifest) == expected_primary,
            "alias_sensitivity_has_30_frozen_rows": len(alias_manifest) == 30,
            "alias_sensitivity_has_six_per_category": Counter(row["category"] for row in alias_manifest) == expected_alias,
            "union_has_30_physical_assets": len(physical) == 30,
            "selection_hashes_recompute": all(
                row["selection_hash"]
                == sha256_bytes(row["rank_payload"].encode("utf-8"))
                for rows in selection["panels"].values() for row in rows
            ),
            "shared_categories_have_identical_ordered_selection": all(
                [row["identity"] for row in primary_manifest if row["category"] == category]
                == [row["identity"] for row in alias_manifest if row["category"] == category]
                for category in PRIMARY_CATEGORIES
            ),
            "reference_protocol_snapshot_semantically_matches": json.loads(
                protocol_snapshot_path.read_text(encoding="utf-8")
            )
            == reference_protocol,
            "strict_panel_missing_exact_table": audit["strict_five_category_exact_panel_feasible"] is False,
            "manifest_root_verification_passed": source_layout["passed"] is True,
            "hf_revision_verification_passed": hf_revision_evidence["passed"] is True,
            "primary_has_no_table_alias": all(row["raw_category"] not in TABLE_ALIAS_SENSITIVITY_ALLOWLIST for row in primary_manifest),
            "alias_table_uses_only_prespecified_allowlist": all(
                row["raw_category"] in TABLE_ALIAS_SENSITIVITY_ALLOWLIST
                for row in alias_manifest if row["category"] == "table"
            ),
            "no_replacement": all(record["failure_replaced"] is False for record in physical.values()),
            "articulation_annotations_not_used_as_gold": all(
                row["artiverse_articulation_annotation_used_for_roles"] is False
                for panel in (PRIMARY_PANEL, ALIAS_PANEL)
                for row in jsonl_read(output / f"{panel}_urdf_name_only_alignment_records.jsonl")
            ),
            "extracted_hashes_recompute": all(
                all(sha256_file(output / value["path"]) == value["sha256"] for value in row.get("files", {}).values())
                for row in physical.values()
            ),
            "summary_selection_hash_matches": summary["hashes"]["frozen_selection_sha256"] == sha256_file(local_selection_path),
        },
        "runner_sha256": summary["hashes"]["runner_sha256"],
    }
    verification["passed"] = all(verification["checks"].values())
    write_json(output / "verification.json", verification)
    if not verification["passed"]:
        raise ValueError(f"verification failed: {verification['checks']}")
    print(
        json.dumps(
            {
                "output": str(output),
                "primary_requested": 24,
                "primary_available": p_structure["available_count"],
                "primary_valid_trees": p_structure["valid_tree_count"],
                "alias_requested": 30,
                "alias_available": a_structure["available_count"],
                "alias_valid_trees": a_structure["valid_tree_count"],
                "paper_ready": False,
            },
            indent=2,
        )
    )
    return 0


def jsonl_read(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


if __name__ == "__main__":
    raise SystemExit(main())
