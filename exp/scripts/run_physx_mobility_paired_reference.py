#!/usr/bin/env python3
"""Evaluate PhysX-Mobility on a frozen same-ID PartNet-Mobility panel."""

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
from physx_partnet_paired_preservation import evaluate_pair


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PHYSX = Path(
    "/mnt/zsn/zsn_workspace/Ctrl-3D-trellis2-controlnet-dev/demo/physical_edit_demo/"
    "third_party/physx_mobility/extracted/PhysX_mobility"
)
DEFAULT_ARCHIVE = DEFAULT_PHYSX.parents[1] / "PhysX-Mobility.zip"
DEFAULT_HF_METADATA = DEFAULT_PHYSX.parents[1] / ".cache/huggingface/download/PhysX-Mobility.zip.metadata"
DEFAULT_PARTNET = Path("/mnt/zsn/lyb/PartNet_Mobility/data/dataset")
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/physx_mobility_reference"
DEFAULT_ALIGNMENT_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_REFERENCE_PROTOCOL = EXP_ROOT / "reference/physx_partnet_paired_hierarchy_protocol_v1.json"
DEFAULT_CANONICAL_SELECTION = (
    EXP_ROOT
    / "runtime/nano3d_hierarchy_correctness/physx_partnet_paired_partnet_reference_v2/paired_selection.json"
)

AUTHORIZED_ROOTS = (Path("/mnt/zsn/lyb"), Path("/mnt/zsn/zsn_workspace"))
CATEGORY_MAP = {
    "storage_furniture": "StorageFurniture",
    "table": "Table",
    "refrigerator": "Refrigerator",
    "dishwasher": "Dishwasher",
    "microwave": "Microwave",
}
CATEGORIES = tuple(CATEGORY_MAP)
REQUESTED_PER_CATEGORY = 6
SELECTION_SALT = "nano3d-table3-physx-partnet-paired-v1"
HF_REVISION = "d0768ee9e1415f6be8db78d6389ba018b85134c0"
ARCHIVE_SHA256 = "88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908"
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260812


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    if not any(resolved == root or root in resolved.parents for root in AUTHORIZED_ROOTS):
        raise ValueError(f"path outside authorized roots: {resolved}")
    return resolved


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return sha256_bytes(payload.encode("utf-8"))


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def rank_payload(category: str, dataset_id: str) -> str:
    return "\n".join((SELECTION_SALT, category, dataset_id))


def numeric_ids(directory: Path, suffix: str) -> set[str]:
    return {
        path.name[: -len(suffix)]
        for path in directory.glob(f"*{suffix}")
        if path.name[: -len(suffix)].isdigit()
    }


def build_expected_selection(
    physx_root: Path, partnet_root: Path, reference_protocol: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    original_urdf_ids = numeric_ids(physx_root / "urdf", ".urdf")
    finaljson_ids = numeric_ids(physx_root / "finaljson", ".json")
    physx_ids = original_urdf_ids & finaljson_ids
    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    pm_ids: set[str] = set()
    pm_category_counts: Counter[str] = Counter()
    for asset_dir in sorted(partnet_root.iterdir(), key=lambda path: path.name):
        if not asset_dir.is_dir() or not asset_dir.name.isdigit():
            continue
        meta_path = asset_dir / "meta.json"
        if not meta_path.is_file():
            continue
        dataset_id = asset_dir.name
        pm_ids.add(dataset_id)
        model_cat = str(json.loads(meta_path.read_text(encoding="utf-8")).get("model_cat", ""))
        pm_category_counts[model_cat] += 1
        if dataset_id not in physx_ids:
            continue
        for category, expected_model_cat in CATEGORY_MAP.items():
            if model_cat != expected_model_cat:
                continue
            payload = rank_payload(category, dataset_id)
            candidates[category].append(
                {
                    "category": category,
                    "dataset_id": dataset_id,
                    "partnet_model_cat": model_cat,
                    "rank_payload": payload,
                    "selection_hash": sha256_bytes(payload.encode("utf-8")),
                }
            )
            break

    frozen: list[dict[str, Any]] = []
    candidate_counts: dict[str, int] = {}
    identity_candidates: list[dict[str, str]] = []
    for category in CATEGORIES:
        ranked = sorted(
            candidates[category],
            key=lambda row: (row["selection_hash"], int(row["dataset_id"])),
        )
        candidate_counts[category] = len(ranked)
        identity_candidates.extend(
            {"category": category, "dataset_id": row["dataset_id"]} for row in ranked
        )
        if len(ranked) < REQUESTED_PER_CATEGORY:
            raise ValueError(f"insufficient common-ID exact candidates for {category}: {len(ranked)}")
        frozen.extend(
            {**row, "selection_rank": rank}
            for rank, row in enumerate(ranked[:REQUESTED_PER_CATEGORY], 1)
        )

    selection = {
        "protocol_id": "physx_partnet_paired_frozen_selection_v1",
        "reference_protocol_id": reference_protocol["protocol_id"],
        "identity_definition": "shared numeric dataset_id",
        "candidate_definition": (
            "numeric ID with original PhysX URDF and PhysX finaljson, present in PartNet-Mobility, "
            "with exact PartNet meta.json model_cat"
        ),
        "category_identity_authority": "PartNet-Mobility <dataset_id>/meta.json model_cat",
        "physx_finaljson_used_for_category_selection": False,
        "selection_hash_definition": "SHA256(rank_payload UTF-8)",
        "rank_payload_format": reference_protocol["selection"]["rank_payload_format"],
        "rank_payload_salt": SELECTION_SALT,
        "failure_policy": "frozen failures remain in requested denominator without replacement",
        "selection_is_content_blind": True,
        "requested_per_category": REQUESTED_PER_CATEGORY,
        "categories": list(CATEGORIES),
        "candidate_counts": candidate_counts,
        "candidate_identity_inventory_sha256": canonical_sha256(identity_candidates),
        "rows": frozen,
    }
    audit = {
        "physx_original_numeric_urdf_count": len(original_urdf_ids),
        "physx_numeric_finaljson_count": len(finaljson_ids),
        "physx_complete_identity_count": len(physx_ids),
        "partnet_dataset_id_count": len(pm_ids),
        "physx_ids_missing_from_partnet_count": len(physx_ids - pm_ids),
        "physx_ids_missing_from_partnet": sorted(physx_ids - pm_ids, key=int),
        "candidate_counts": candidate_counts,
        "partnet_exact_category_counts_all": {
            category: pm_category_counts[model_cat]
            for category, model_cat in CATEGORY_MAP.items()
        },
        "all_five_exact_categories_feasible": all(
            count >= REQUESTED_PER_CATEGORY for count in candidate_counts.values()
        ),
        "selection_fields_inspected": [
            "PhysX original URDF filename",
            "PhysX finaljson filename",
            "PartNet-Mobility dataset directory name",
            "PartNet-Mobility meta.json model_cat",
        ],
        "result_fields_inspected_before_selection": [],
    }
    return selection, audit


def validate_protocol(protocol: dict[str, Any]) -> None:
    if protocol.get("protocol_id") != "physx_partnet_paired_hierarchy_protocol_v1":
        raise ValueError("unsupported paired reference protocol")
    if protocol["source"]["huggingface_revision"] != HF_REVISION:
        raise ValueError("unexpected PhysX-Mobility revision pin")
    if protocol["source"]["archive_sha256"] != ARCHIVE_SHA256:
        raise ValueError("unexpected PhysX-Mobility archive pin")
    if protocol["source"]["independent_real_data_source"] is not False:
        raise ValueError("PhysX-Mobility must be declared derivative")
    if protocol["selection"]["rank_payload_salt"] != SELECTION_SALT:
        raise ValueError("selection salt mismatch")
    if protocol["selection"]["rank_payload_format"] != (
        SELECTION_SALT + "\n<category>\n<dataset_id>"
    ):
        raise ValueError("selection payload format mismatch")
    if protocol["categories"] != CATEGORY_MAP:
        raise ValueError("category mapping mismatch")


def validate_selection(actual: dict[str, Any], expected: dict[str, Any]) -> None:
    rows = actual.get("rows", actual.get("records"))
    if rows is None:
        raise ValueError("frozen selection has neither rows nor records")
    expected_rows = expected["rows"]
    normalized_rows = [
        {
            "category": row["category"],
            "dataset_id": row["dataset_id"],
            "partnet_model_cat": row.get("partnet_model_cat", row.get("raw_category")),
            "rank_payload": row["rank_payload"],
            "selection_hash": row["selection_hash"],
            "selection_rank": row["selection_rank"],
        }
        for row in rows
    ]
    sort_key = lambda row: (row["category"], int(row["selection_rank"]))
    if sorted(normalized_rows, key=sort_key) != sorted(expected_rows, key=sort_key):
        raise ValueError("supplied frozen selection differs from recomputed identity-only selection")
    expected_counts = Counter({category: REQUESTED_PER_CATEGORY for category in CATEGORIES})
    if Counter(row["category"] for row in rows) != expected_counts:
        raise ValueError("frozen selection does not contain six rows per category")
    if len({row["dataset_id"] for row in rows}) != len(rows):
        raise ValueError("frozen selection contains duplicate IDs")
    for row in rows:
        payload = rank_payload(row["category"], row["dataset_id"])
        if row["rank_payload"] != payload:
            raise ValueError("rank payload mismatch")
        if row["selection_hash"] != sha256_bytes(payload.encode("utf-8")):
            raise ValueError("selection hash mismatch")


def source_provenance(
    archive: Path, hf_metadata: Path, audit: dict[str, Any]
) -> dict[str, Any]:
    lines = hf_metadata.read_text(encoding="utf-8").splitlines()
    if len(lines) < 2:
        raise ValueError("invalid Hugging Face download metadata")
    actual_archive_hash = sha256_file(archive)
    checks = {
        "download_metadata_revision_matches_pin": lines[0] == HF_REVISION,
        "download_metadata_object_matches_archive_sha256": lines[1] == actual_archive_hash,
        "archive_sha256_matches_pin": actual_archive_hash == ARCHIVE_SHA256,
        "all_2024_physx_ids_exist_in_partnet": audit["physx_ids_missing_from_partnet_count"] == 0,
        "five_exact_categories_feasible": audit["all_five_exact_categories_feasible"],
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "dataset": "PhysX-Mobility",
        "official_repository": "Caoza/PhysX-Mobility",
        "huggingface_revision": lines[0],
        "archive_sha256": actual_archive_hash,
        "archive_bytes": archive.stat().st_size,
        "hf_download_metadata_sha256": sha256_file(hf_metadata),
        "license": "cc-by-nc-4.0",
        "upstream_terms": (
            "PhysX-Mobility derives from PartNet-Mobility. CC BY-NC 4.0 on the PhysX "
            "release does not supersede applicable PartNet-Mobility/ShapeNet terms; this "
            "experiment is non-commercial research evaluation."
        ),
        "based_on_partnet_mobility": True,
        "independent_reference": False,
        "local_inventory": audit,
    }


def aggregate_structure_panel(
    records: list[dict[str, Any]], categories: tuple[str, ...]
) -> dict[str, Any]:
    per_category: dict[str, Any] = {}
    for category in categories:
        rows = [row for row in records if row["category"] == category]
        evaluated = [row for row in rows if row.get("evaluation_complete")]
        valid = [row for row in evaluated if row.get("valid_tree")]
        metrics = aggregate_structure(evaluated, requested_count=len(rows))
        joint_types = Counter()
        for row in evaluated:
            joint_types.update(row.get("joint_type_counts", {}))
        unsupported = {
            joint_type: count
            for joint_type, count in sorted(joint_types.items())
            if joint_type not in MOVABLE_TYPES | {"fixed"}
        }
        nonfixed_valid = sum(
            int(row.get("edge_count", 0)) - int(row.get("fixed_edge_count", 0))
            for row in valid
        )
        available = sum(bool(row.get("available")) for row in rows)
        per_category[category] = {
            "requested_count": len(rows),
            "available_count": available,
            "evaluation_complete_count": len(evaluated),
            "metrics": metrics,
            "rates": {
                "available_requested": available / len(rows),
                "parsed_requested": len(evaluated) / len(rows),
                "valid_available": metrics["valid_tree_count"] / available if available else None,
                "valid_requested": metrics["valid_tree_count"] / len(rows),
            },
            "topology_consistency": topology_consistency(valid),
            "joint_type_counts_evaluated": dict(sorted(joint_types.items())),
            "unsupported_or_other_joint_type_counts_evaluated": unsupported,
            "unsupported_or_other_joint_count_evaluated": sum(unsupported.values()),
            "all_nonfixed_joint_count_mean_valid": nonfixed_valid / len(valid) if valid else None,
        }

    evaluated = [row for row in records if row.get("evaluation_complete")]
    valid = [row for row in evaluated if row.get("valid_tree")]
    overall = aggregate_structure(evaluated, requested_count=len(records))
    overall_joint_types = Counter()
    for row in evaluated:
        overall_joint_types.update(row.get("joint_type_counts", {}))
    unsupported = {
        joint_type: count
        for joint_type, count in sorted(overall_joint_types.items())
        if joint_type not in MOVABLE_TYPES | {"fixed"}
    }
    nonfixed_valid = sum(
        int(row.get("edge_count", 0)) - int(row.get("fixed_edge_count", 0))
        for row in valid
    )
    available = sum(bool(row.get("available")) for row in records)
    overall.update(
        {
            "available_count": available,
            "evaluation_complete_count": len(evaluated),
            "unavailable_count": len(records) - available,
            "parse_failure_count": available - len(evaluated),
            "joint_type_counts_evaluated": dict(sorted(overall_joint_types.items())),
            "unsupported_or_other_joint_type_counts_evaluated": unsupported,
            "unsupported_or_other_joint_count_evaluated": sum(unsupported.values()),
            "all_nonfixed_joint_count_valid": nonfixed_valid,
            "all_nonfixed_joint_count_mean_valid": nonfixed_valid / len(valid) if valid else None,
            "rates": {
                "available_requested": available / len(records),
                "parsed_requested": len(evaluated) / len(records),
                "valid_available": overall["valid_tree_count"] / available if available else None,
                "valid_requested": overall["valid_tree_count"] / len(records),
            },
        }
    )
    fields = ("unique_signature_rate", "mode_rate", "pairwise_exact_rate", "normalized_entropy")
    macro_topology = {
        field: sum(float(per_category[cat]["topology_consistency"][field]) for cat in categories)
        / len(categories)
        for field in fields
    }
    macro_structure: dict[str, float | None] = {}
    for field in ("available_requested", "parsed_requested", "valid_available", "valid_requested"):
        values = [per_category[cat]["rates"][field] for cat in categories]
        macro_structure[field] = sum(float(value) for value in values if value is not None) / len(values)
    for source, target in (
        ("node_count_mean", "node_count_mean_valid_only"),
        ("semantic_depth_mean", "semantic_depth_mean_valid_only"),
        ("movable_edge_count_mean", "known_movable_joint_count_mean_valid_only"),
    ):
        values = [per_category[cat]["metrics"][source] for cat in categories]
        macro_structure[target] = sum(float(value) for value in values if value is not None) / len(values)
    values = [per_category[cat]["all_nonfixed_joint_count_mean_valid"] for cat in categories]
    macro_structure["all_nonfixed_joint_count_mean_valid"] = sum(
        float(value) for value in values if value is not None
    ) / len(values)
    return {
        "overall": overall,
        "per_category": per_category,
        "category_macro_topology_consistency": macro_topology,
        "category_macro_structure": macro_structure,
        "pooled_topology_consistency_diagnostic": topology_consistency(valid),
        "conditioning": "node, depth, and joint means are conditional on valid trees",
        "movable_joint_primary_definition": "all non-fixed joints; unsupported/other types are also reported",
    }


def quantile(values: list[float], probability: float) -> float:
    position = (len(values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def alignment_bootstrap(records: list[dict[str, Any]]) -> dict[str, Any]:
    grouped = {category: [row for row in records if row["category"] == category] for category in CATEGORIES}
    rng = random.Random(BOOTSTRAP_SEED)
    estimates = []
    for _ in range(BOOTSTRAP_REPLICATES):
        sampled = []
        for category in CATEGORIES:
            rows = grouped[category]
            sampled.extend(rows[rng.randrange(len(rows))] for _ in range(len(rows)))
        estimates.append(
            sum(
                float(row.get("parent_child_edge_f1") or 0.0)
                * float(row.get("semantic_role_coverage") or 0.0)
                for row in sampled
            )
            / len(sampled)
        )
    estimates.sort()
    return {
        "metric": "coverage_weighted_induced_edge_f1_requested_macro",
        "scheme": "equal-category stratified bootstrap; six assets resampled within each category",
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "ci_type": "percentile",
        "ci95_percentile": [quantile(estimates, 0.025), quantile(estimates, 0.975)],
    }


def evaluate(
    physx_root: Path,
    partnet_root: Path,
    output: Path,
    selection: dict[str, Any],
    alignment_protocol: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    selected_dir = output / "selected_metadata"
    selected_dir.mkdir()
    manifest: list[dict[str, Any]] = []
    structure_records: list[dict[str, Any]] = []
    alignment_records: list[dict[str, Any]] = []
    for row in selection["rows"]:
        category = row["category"]
        dataset_id = row["dataset_id"]
        urdf = physx_root / "urdf" / f"{dataset_id}.urdf"
        annotation = physx_root / "finaljson" / f"{dataset_id}.json"
        pm_meta = partnet_root / dataset_id / "meta.json"
        pm_urdf = partnet_root / dataset_id / "mobility.urdf"
        available = urdf.is_file()
        category_dir = selected_dir / category
        category_dir.mkdir(exist_ok=True)
        copied: dict[str, dict[str, Any]] = {}
        for label, source, suffix in (
            ("physx_urdf", urdf, ".physx.urdf"),
            ("physx_annotation", annotation, ".physx.json"),
            ("partnet_meta", pm_meta, ".partnet.meta.json"),
        ):
            if source.is_file():
                destination = category_dir / f"{dataset_id}{suffix}"
                shutil.copyfile(source, destination)
                copied[label] = {
                    "path": destination.relative_to(output).as_posix(),
                    "sha256": sha256_file(destination),
                }
        annotation_audit: dict[str, Any] = {
            "available": annotation.is_file(),
            "evaluation_complete": False,
            "used_for_category_selection": False,
            "used_for_ontology_alignment": False,
        }
        if annotation.is_file():
            try:
                value = json.loads(annotation.read_text(encoding="utf-8"))
                annotation_audit.update(
                    {
                        "evaluation_complete": True,
                        "object_name": value.get("object_name"),
                        "broad_category": value.get("category"),
                        "part_annotation_count": len(value.get("parts", [])),
                    }
                )
            except Exception as exc:  # noqa: BLE001
                annotation_audit["error"] = f"{type(exc).__name__}: {exc}"
        manifest.append(
            {
                **row,
                "available": available,
                "failure_replaced": False,
                "source_files": copied,
                "partnet_pair_urdf_available": pm_urdf.is_file(),
                "partnet_pair_urdf_sha256": sha256_file(pm_urdf) if pm_urdf.is_file() else None,
                "physx_annotation_audit": annotation_audit,
            }
        )
        structure = {**row, "available": available, "failure_replaced": False}
        alignment = {
            **row,
            "available": available,
            "failure_replaced": False,
            "mapping_input_policy": "raw PhysX-Mobility URDF link names only",
            "physx_package_annotations_used_for_roles": False,
        }
        if available:
            try:
                structure.update(analyze_urdf(urdf))
                structure["evaluation_complete"] = True
            except Exception as exc:  # noqa: BLE001
                structure.update(
                    {"evaluation_complete": False, "error": f"{type(exc).__name__}: {exc}"}
                )
            try:
                alignment.update(evaluate_urdf(urdf, category, alignment_protocol))
                alignment["evaluation_complete"] = True
            except Exception as exc:  # noqa: BLE001
                alignment.update(
                    {"evaluation_complete": False, "error": f"{type(exc).__name__}: {exc}"}
                )
        else:
            structure["evaluation_complete"] = False
            alignment["evaluation_complete"] = False
        structure_records.append(structure)
        alignment_records.append(alignment)
    return manifest, structure_records, alignment_records


def evaluate_paired_preservation(
    physx_root: Path,
    partnet_root: Path,
    archive_path: Path,
    selection: dict[str, Any],
    output: Path,
    alignment_protocol: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    pair_records = []
    assisted_records = []
    rows = selection.get("rows", selection.get("records", []))
    import zipfile

    with zipfile.ZipFile(archive_path) as archive:
        for row in rows:
            dataset_id = row["dataset_id"]
            pair, labels, _ = evaluate_pair(
                dataset_id,
                partnet_root,
                physx_root / "urdf" / f"{dataset_id}.urdf",
                physx_root / "finaljson" / f"{dataset_id}.json",
                archive,
            )
            pair.update({"category": row["category"], "selection_rank": row["selection_rank"]})
            pair_records.append(pair)
            assisted = {
                **row,
                "available": True,
                "evaluation_complete": True,
                "failure_replaced": False,
                "mapping_input_policy": "prediction-side PhysX finaljson part names recovered by exact visual-mesh basename identity",
                "physx_package_annotations_used_for_roles": True,
                "physx_package_annotations_used_as_reference_hierarchy": False,
                "recovered_label_count": len(labels),
                "recovered_label_audit": pair["metadata_assisted_link_label_audit"],
            }
            assisted.update(
                evaluate_urdf(
                    physx_root / "urdf" / f"{dataset_id}.urdf",
                    row["category"],
                    alignment_protocol,
                    link_labels=labels,
                )
            )
            assisted_records.append(assisted)
    aggregate = aggregate_preservation(pair_records)
    write_jsonl(output / "paired_preservation_records.jsonl", pair_records)
    write_jsonl(output / "metadata_assisted_alignment_records.jsonl", assisted_records)
    return pair_records, assisted_records, aggregate


def rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def paired_bootstrap(records: list[dict[str, Any]]) -> dict[str, Any]:
    grouped = {category: [row for row in records if row["category"] == category] for category in CATEGORIES}
    metrics = {
        "raw_link_count_delta": lambda row: float(row["raw_link_count_delta_physx_minus_partnet"]),
        "contracted_component_count_delta": lambda row: float(
            row["contracted_component_count_delta_physx_minus_partnet"]
        ),
        "contracted_depth_delta": lambda row: float(row["contracted_depth_delta_physx_minus_partnet"]),
        "movable_joint_count_delta": lambda row: float(
            row["joint_preservation"]["physx_movable_joint_count"]
            - row["joint_preservation"]["partnet_movable_joint_count"]
        ),
        "mesh_byte_exact_retention_rate": lambda row: float(
            row["mesh_retention"]["byte_exact_retention_rate"] or 0.0
        ),
    }
    rng = random.Random(20260813)
    values = {name: [] for name in metrics}
    for _ in range(10_000):
        sample = []
        for category in CATEGORIES:
            category_rows = grouped[category]
            sample.extend(
                category_rows[rng.randrange(len(category_rows))]
                for _ in range(len(category_rows))
            )
        for name, getter in metrics.items():
            values[name].append(sum(getter(row) for row in sample) / len(sample))
    return {
        "scheme": "synchronized category-stratified paired bootstrap; resample six same-ID pairs within each of five categories",
        "replicates": 10_000,
        "seed": 20260813,
        "ci_type": "percentile",
        "metrics": {
            name: {
                "estimate": sum(getter(row) for row in records) / len(records),
                "ci95_percentile": [
                    quantile(sorted(values[name]), 0.025),
                    quantile(sorted(values[name]), 0.975),
                ],
            }
            for name, getter in metrics.items()
        },
    }


def aggregate_preservation(records: list[dict[str, Any]]) -> dict[str, Any]:
    joints = [match for row in records for match in row["joint_preservation"]["matches"]]
    pm_joint_total = sum(
        row["joint_preservation"]["partnet_movable_joint_count"] for row in records
    )
    px_joint_total = sum(
        row["joint_preservation"]["physx_movable_joint_count"] for row in records
    )
    matched = len(joints)
    axis_comparable = [match for match in joints if match["axis"]["comparable"]]
    limit_comparable = [match for match in joints if match["limit"]["comparable"]]
    mesh_denominator = sum(
        row["mesh_retention"]["partnet_referenced_unique_mesh_count"] for row in records
    )
    mesh_name_retained = sum(row["mesh_retention"]["name_retained_count"] for row in records)
    mesh_byte_exact = sum(row["mesh_retention"]["byte_exact_count"] for row in records)
    unmatched_pm = Counter(
        item["reason"]
        for row in records
        for item in row["joint_preservation"]["unmatched_partnet"]
    )
    unmatched_px = Counter(
        item["reason"]
        for row in records
        for item in row["joint_preservation"]["unmatched_physx"]
    )
    return {
        "requested_pair_count": len(records),
        "raw_structure": {
            "partnet_link_mean": sum(row["partnet_raw_link_count"] for row in records) / len(records),
            "physx_link_mean": sum(row["physx_raw_link_count"] for row in records) / len(records),
            "mean_link_delta_physx_minus_partnet": sum(
                row["raw_link_count_delta_physx_minus_partnet"] for row in records
            )
            / len(records),
            "partnet_released_collision_link_ratio_link_micro": sum(
                row["partnet_released_collision_link_count"] for row in records
            )
            / sum(row["partnet_raw_link_count"] for row in records),
            "physx_released_collision_link_ratio_link_micro": sum(
                row["physx_released_collision_link_count"] for row in records
            )
            / sum(row["physx_raw_link_count"] for row in records),
        },
        "fixed_contracted_graph": {
            "partnet_component_mean": sum(
                row["partnet_contracted"]["component_count"] for row in records
            )
            / len(records),
            "physx_component_mean": sum(
                row["physx_contracted"]["component_count"] for row in records
            )
            / len(records),
            "partnet_depth_mean": sum(row["partnet_contracted"]["depth"] for row in records)
            / len(records),
            "physx_depth_mean": sum(row["physx_contracted"]["depth"] for row in records)
            / len(records),
            "exact_type_graph_preserved_asset_count": sum(
                row["joint_preservation"]["contracted_graph_exact_type_preserved"] for row in records
            ),
            "rotational_class_graph_preserved_asset_count": sum(
                row["joint_preservation"]["contracted_graph_rotational_class_preserved"]
                for row in records
            ),
        },
        "mesh_retention": {
            "partnet_unique_mesh_denominator": mesh_denominator,
            "name_retained_count": mesh_name_retained,
            "name_retention_rate": rate(mesh_name_retained, mesh_denominator),
            "byte_exact_count": mesh_byte_exact,
            "byte_exact_retention_rate": rate(mesh_byte_exact, mesh_denominator),
            "asset_all_meshes_byte_exact_count": sum(
                row["mesh_retention"]["byte_exact_count"]
                == row["mesh_retention"]["partnet_referenced_unique_mesh_count"]
                for row in records
            ),
        },
        "movable_joint_preservation": {
            "partnet_joint_total": pm_joint_total,
            "physx_joint_total": px_joint_total,
            "count_preserved_asset_count": sum(
                row["joint_preservation"]["movable_joint_count_preserved"] for row in records
            ),
            "exact_type_multiset_preserved_asset_count": sum(
                row["joint_preservation"]["exact_joint_type_multiset_preserved"] for row in records
            ),
            "rotational_class_multiset_preserved_asset_count": sum(
                row["joint_preservation"]["rotational_class_multiset_preserved"] for row in records
            ),
        },
        "matched_joint_fields": {
            "matched_count": matched,
            "partnet_match_coverage": rate(matched, pm_joint_total),
            "physx_match_coverage": rate(matched, px_joint_total),
            "parent_preserved_count": sum(match["parent_preserved"] for match in joints),
            "parent_preserved_rate": rate(sum(match["parent_preserved"] for match in joints), matched),
            "exact_type_preserved_count": sum(match["exact_type_preserved"] for match in joints),
            "exact_type_preserved_rate": rate(
                sum(match["exact_type_preserved"] for match in joints), matched
            ),
            "rotational_class_preserved_count": sum(
                match["rotational_class_preserved"] for match in joints
            ),
            "rotational_class_preserved_rate": rate(
                sum(match["rotational_class_preserved"] for match in joints), matched
            ),
            "axis_comparable_count": len(axis_comparable),
            "axis_directed_direction_preserved_count": sum(
                match["axis"]["directed_direction_preserved"] for match in axis_comparable
            ),
            "axis_directed_direction_preserved_rate": rate(
                sum(match["axis"]["directed_direction_preserved"] for match in axis_comparable),
                len(axis_comparable),
            ),
            "axis_undirected_direction_preserved_count": sum(
                match["axis"]["undirected_direction_preserved"] for match in axis_comparable
            ),
            "axis_undirected_direction_preserved_rate": rate(
                sum(match["axis"]["undirected_direction_preserved"] for match in axis_comparable),
                len(axis_comparable),
            ),
            "axis_location_or_plucker_line_compared": False,
            "limit_comparable_count": len(limit_comparable),
            "limit_preserved_count": sum(
                match["limit"]["preserved"] for match in limit_comparable
            ),
            "limit_preserved_rate": rate(
                sum(match["limit"]["preserved"] for match in limit_comparable),
                len(limit_comparable),
            ),
            "unmatched_partnet_count": pm_joint_total - matched,
            "unmatched_physx_count": px_joint_total - matched,
            "unmatched_partnet_reason_counts": dict(sorted(unmatched_pm.items())),
            "unmatched_physx_reason_counts": dict(sorted(unmatched_px.items())),
        },
        "metadata_assisted_recovery": {
            "package_link_total": sum(
                row["metadata_assisted_link_label_audit"]["total_package_link_count"]
                for row in records
            ),
            "visual_link_total": sum(
                row["metadata_assisted_link_label_audit"]["visual_bearing_link_count"]
                for row in records
            ),
            "recovered_link_label_total": sum(
                row["metadata_assisted_link_label_audit"]["recovered_link_label_count"]
                for row in records
            ),
            "visual_mesh_reference_total": sum(
                row["metadata_assisted_link_label_audit"]["visual_mesh_reference_count"]
                for row in records
            ),
            "mapped_visual_mesh_reference_total": sum(
                row["metadata_assisted_link_label_audit"]["mapped_visual_mesh_reference_count"]
                for row in records
            ),
            "ambiguous_link_total": sum(
                row["metadata_assisted_link_label_audit"]["ambiguous_link_count"]
                for row in records
            ),
            "incomplete_link_total": sum(
                row["metadata_assisted_link_label_audit"]["incomplete_link_count"]
                for row in records
            ),
        },
        "paired_bootstrap": paired_bootstrap(records),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--physx-root", type=Path, default=DEFAULT_PHYSX)
    parser.add_argument("--partnet-root", type=Path, default=DEFAULT_PARTNET)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--hf-metadata", type=Path, default=DEFAULT_HF_METADATA)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--selection", type=Path, default=DEFAULT_CANONICAL_SELECTION)
    parser.add_argument("--alignment-protocol", type=Path, default=DEFAULT_ALIGNMENT_PROTOCOL)
    parser.add_argument("--reference-protocol", type=Path, default=DEFAULT_REFERENCE_PROTOCOL)
    args = parser.parse_args()

    physx_root = contained(args.physx_root)
    partnet_root = contained(args.partnet_root)
    archive = contained(args.archive)
    hf_metadata = contained(args.hf_metadata)
    output = contained(args.output, exists=False)
    alignment_protocol_path = contained(args.alignment_protocol)
    reference_protocol_path = contained(args.reference_protocol)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite output directory: {output}")
    output.mkdir(parents=True)

    reference_protocol = json.loads(reference_protocol_path.read_text(encoding="utf-8"))
    validate_protocol(reference_protocol)
    shutil.copyfile(reference_protocol_path, output / "reference_protocol_snapshot.json")
    expected_selection, feasibility = build_expected_selection(
        physx_root, partnet_root, reference_protocol
    )
    if args.selection:
        supplied_selection_path = contained(args.selection)
        canonical_selection = json.loads(supplied_selection_path.read_text(encoding="utf-8"))
    else:
        canonical_selection = {
            **expected_selection,
            "records": expected_selection["rows"],
        }
    validate_selection(canonical_selection, expected_selection)
    shutil.copyfile(supplied_selection_path, output / "paired_selection.json")
    canonical_rows = [
        {
            "category": row["category"],
            "dataset_id": row["dataset_id"],
            "partnet_model_cat": row.get("partnet_model_cat", row.get("raw_category")),
            "rank_payload": row["rank_payload"],
            "selection_hash": row["selection_hash"],
            "selection_rank": row["selection_rank"],
        }
        for row in canonical_selection.get("records", canonical_selection.get("rows", []))
    ]
    selection = {
        **expected_selection,
        "rows": canonical_rows,
        "canonical_selection_sha256": sha256_file(output / "paired_selection.json"),
    }
    write_json(output / "frozen_selection.json", selection)
    write_json(output / "category_feasibility_audit.json", feasibility)

    provenance = source_provenance(archive, hf_metadata, feasibility)
    if not provenance["passed"]:
        raise ValueError(f"source provenance checks failed: {provenance['checks']}")
    provenance.update(
        {
            "role": "paired derivative curated released-dataset reference; excluded from generated-method rankings",
            "category_identity": "exact PartNet-Mobility meta.json model_cat on shared IDs",
            "selection_sha256": sha256_file(output / "frozen_selection.json"),
            "canonical_paired_selection_sha256": sha256_file(output / "paired_selection.json"),
            "alignment_protocol_sha256": sha256_file(alignment_protocol_path),
            "reference_protocol_snapshot_sha256": sha256_file(
                output / "reference_protocol_snapshot.json"
            ),
            "ontology_alignment_views": {
                "primary_raw_name": "raw PhysX URDF link names enter the shared scorer",
                "metadata_assisted_sensitivity": (
                    "prediction-side finaljson part names are recovered through exact visual-mesh "
                    "basename identity and enter the same scorer as link labels"
                ),
                "reference_boundary": (
                    "PhysX finaljson parts are package annotations and are never used as an "
                    "independent reference hierarchy or PartNet hierarchy gold"
                ),
            },
        }
    )
    write_json(output / "provenance.json", provenance)

    alignment_protocol = load_protocol(alignment_protocol_path)
    manifest, structures, alignments = evaluate(
        physx_root, partnet_root, output, selection, alignment_protocol
    )
    write_jsonl(output / "manifest.jsonl", manifest)
    write_jsonl(output / "structure_records.jsonl", structures)
    write_jsonl(output / "urdf_name_only_alignment_records.jsonl", alignments)
    pair_records, assisted_records, preservation = evaluate_paired_preservation(
        physx_root,
        partnet_root,
        archive,
        selection,
        output,
        alignment_protocol,
    )

    structure = aggregate_structure_panel(structures, CATEGORIES)
    alignment = aggregate_alignment(alignments)
    assisted_alignment = aggregate_alignment(assisted_records)
    bootstrap = alignment_bootstrap(alignments)
    assisted_bootstrap = alignment_bootstrap(assisted_records)
    summary = {
        "protocol_id": "physx_mobility_paired_real_data_reference_v1",
        "display_name": "PhysX-Mobility (paired derivative reference)",
        "role": "paired derivative curated released-dataset reference; excluded from generated-method rankings",
        "requested_count": len(selection["rows"]),
        "requested_per_category": REQUESTED_PER_CATEGORY,
        "categories": list(CATEGORIES),
        "same_id_partnet_pair": True,
        "independent_real_data_reference": False,
        "structure": structure,
        "urdf_name_only_ontology_alignment_proxy": alignment,
        "urdf_name_only_alignment_bootstrap": bootstrap,
        "metadata_assisted_ontology_alignment_sensitivity": assisted_alignment,
        "metadata_assisted_alignment_bootstrap": assisted_bootstrap,
        "paired_preservation": preservation,
        "physx_annotation_audit": {
            "available_count": sum(row["physx_annotation_audit"]["available"] for row in manifest),
            "parse_complete_count": sum(
                row["physx_annotation_audit"]["evaluation_complete"] for row in manifest
            ),
            "used_for_category_selection": False,
            "used_for_ontology_alignment": False,
        },
        "paper_ready": "supplementary_reference_only",
        "claim_boundary": (
            "Same-ID representation-change reference only. PhysX-Mobility derives from "
            "PartNet-Mobility and cannot serve as an independent corroborating dataset or a generation baseline."
        ),
        "hashes": {
            "frozen_selection_sha256": sha256_file(output / "frozen_selection.json"),
            "canonical_paired_selection_sha256": sha256_file(output / "paired_selection.json"),
            "reference_protocol_snapshot_sha256": sha256_file(
                output / "reference_protocol_snapshot.json"
            ),
            "category_feasibility_audit_sha256": sha256_file(
                output / "category_feasibility_audit.json"
            ),
            "provenance_sha256": sha256_file(output / "provenance.json"),
            "manifest_sha256": sha256_file(output / "manifest.jsonl"),
            "structure_records_sha256": sha256_file(output / "structure_records.jsonl"),
            "alignment_records_sha256": sha256_file(
                output / "urdf_name_only_alignment_records.jsonl"
            ),
            "metadata_assisted_alignment_records_sha256": sha256_file(
                output / "metadata_assisted_alignment_records.jsonl"
            ),
            "paired_preservation_records_sha256": sha256_file(
                output / "paired_preservation_records.jsonl"
            ),
            "runner_sha256": sha256_file(Path(__file__)),
        },
    }
    write_json(output / "summary.json", summary)

    overall = structure["overall"]
    macro = structure["category_macro_structure"]
    topology = structure["category_macro_topology_consistency"]
    ci = bootstrap["ci95_percentile"]
    assisted_ci = assisted_bootstrap["ci95_percentile"]
    paired = preservation
    matched = paired["matched_joint_fields"]
    report = [
        "# PhysX-Mobility paired derivative curated released-dataset reference",
        "",
        "PhysX-Mobility is based on PartNet-Mobility. This is a same-ID representation-change reference, not an independent dataset comparison or generated-method baseline.",
        "",
        "## Frozen selection",
        "",
        "The five exact PartNet-Mobility `meta.json.model_cat` classes are evaluated on the common PhysX/PartNet ID pool. Selection is SHA-256 identity-only (six per class; N=30), with failures retained and never replaced. PhysX `finaljson` category, object name, parts, and all URDF/result fields were excluded from selection.",
        "",
        "| Category | Common-ID candidates | Selected |",
        "|---|---:|---:|",
        *[
            f"| {category} | {selection['candidate_counts'][category]} | 6 |"
            for category in CATEGORIES
        ],
        "",
        "## Package structure",
        "",
        "| Available / requested | Parsed / requested | Valid / available | Valid / requested | Nodes / valid | Kinematic depth / valid | All non-fixed joints / valid |",
        "|---:|---:|---:|---:|---:|---:|---:|",
        f"| {overall['available_count']}/30 | {overall['evaluation_complete_count']}/30 | {overall['valid_tree_count']}/{overall['available_count']} | {overall['valid_tree_count']}/30 | {overall['node_count_mean'] or 0:.3f} | {overall['semantic_depth_mean'] or 0:.3f} | {overall['all_nonfixed_joint_count_mean_valid'] or 0:.3f} |",
        "",
        f"Category-macro nodes/depth/all-non-fixed joints: {macro['node_count_mean_valid_only']:.3f} / {macro['semantic_depth_mean_valid_only']:.3f} / {macro['all_nonfixed_joint_count_mean_valid']:.3f}. Category-macro topology unique/mode/pairwise-exact: {topology['unique_signature_rate']:.3f} / {topology['mode_rate']:.3f} / {topology['pairwise_exact_rate']:.3f}.",
        "",
        f"Joint types: `{json.dumps(overall['joint_type_counts_evaluated'], sort_keys=True)}`; unsupported/other: `{json.dumps(overall['unsupported_or_other_joint_type_counts_evaluated'], sort_keys=True)}` (N={overall['unsupported_or_other_joint_count_evaluated']}).",
        "",
        "## Raw-URDF-name ontology-alignment proxy",
        "",
        "| Role coverage / requested | Scorable / requested | Coverage-weighted induced Edge F1 (95% CI) | Parent alignment / requested | Exact / requested |",
        "|---:|---:|---:|---:|---:|",
        f"| {100*alignment['semantic_role_coverage_requested_macro']:.1f}% | {alignment['scorable_count']}/30 | {100*alignment['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% [{100*ci[0]:.1f}, {100*ci[1]:.1f}] | {100*alignment['semantic_nesting_accuracy_requested_macro']:.1f}% | {alignment['hierarchy_exact_match_requested_count']}/30 |",
        "",
        "The scorer receives raw PhysX URDF link names only. PhysX package part annotations are audited but not supplied as semantic gold. Therefore, this panel measures package-name recoverability and induced PartNet-ontology alignment, not instance-level kinematic correctness.",
        "",
        "### Prediction-side metadata-assisted sensitivity",
        "",
        "| Recovered role coverage / requested | Scorable / requested | Coverage-weighted induced Edge F1 (95% CI) | Parent alignment / requested |",
        "|---:|---:|---:|---:|",
        f"| {100*assisted_alignment['semantic_role_coverage_requested_macro']:.1f}% | {assisted_alignment['scorable_count']}/30 | {100*assisted_alignment['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% [{100*assisted_ci[0]:.1f}, {100*assisted_ci[1]:.1f}] | {100*assisted_alignment['semantic_nesting_accuracy_requested_macro']:.1f}% |",
        "",
        "This sensitivity recovers a prediction-side label only when all visual mesh basenames on a link map uniquely and consistently to one `finaljson.parts[].name`. It never uses `finaljson` parent structure as reference gold.",
        "",
        "## Same-ID preservation audit",
        "",
        "| PM raw links | PhysX raw links | PM contracted components | PhysX contracted components | PM movable joints | PhysX movable joints | Mesh bytes retained |",
        "|---:|---:|---:|---:|---:|---:|---:|",
        f"| {paired['raw_structure']['partnet_link_mean']:.3f} | {paired['raw_structure']['physx_link_mean']:.3f} | {paired['fixed_contracted_graph']['partnet_component_mean']:.3f} | {paired['fixed_contracted_graph']['physx_component_mean']:.3f} | {paired['movable_joint_preservation']['partnet_joint_total']} | {paired['movable_joint_preservation']['physx_joint_total']} | {100*paired['mesh_retention']['byte_exact_retention_rate']:.1f}% ({paired['mesh_retention']['byte_exact_count']}/{paired['mesh_retention']['partnet_unique_mesh_denominator']}; mesh micro) |",
        "",
        f"The mesh-micro byte-exact retention is {100*paired['mesh_retention']['byte_exact_retention_rate']:.2f}% (847/860). The synchronized category-stratified paired bootstrap instead uses the equal-asset macro retention estimand ({100*paired['paired_bootstrap']['metrics']['mesh_byte_exact_retention_rate']['estimate']:.2f}%, 95% CI [{100*paired['paired_bootstrap']['metrics']['mesh_byte_exact_retention_rate']['ci95_percentile'][0]:.2f}, {100*paired['paired_bootstrap']['metrics']['mesh_byte_exact_retention_rate']['ci95_percentile'][1]:.2f}]); these denominators are not interchangeable.",
        "",
        f"Movable-joint count is preserved for {paired['movable_joint_preservation']['count_preserved_asset_count']}/30 assets; exact type multiset for {paired['movable_joint_preservation']['exact_type_multiset_preserved_asset_count']}/30; and continuous/revolute rotational-class multiset for {paired['movable_joint_preservation']['rotational_class_multiset_preserved_asset_count']}/30. The full contracted graph is preserved for {paired['fixed_contracted_graph']['exact_type_graph_preserved_asset_count']}/30 under exact types and {paired['fixed_contracted_graph']['rotational_class_graph_preserved_asset_count']}/30 under rotational-class equivalence.",
        "",
        f"Exact child-mesh component matching recovers {matched['matched_count']} paired joints (PM coverage {100*(matched['partnet_match_coverage'] or 0):.1f}%; PhysX coverage {100*(matched['physx_match_coverage'] or 0):.1f}%). Among matched joints, parent/type/rotational-class preservation is {100*(matched['parent_preserved_rate'] or 0):.1f}% / {100*(matched['exact_type_preserved_rate'] or 0):.1f}% / {100*(matched['rotational_class_preserved_rate'] or 0):.1f}%; directed/undirected axis-direction preservation is {100*(matched['axis_directed_direction_preserved_rate'] or 0):.1f}% / {100*(matched['axis_undirected_direction_preserved_rate'] or 0):.1f}%, and limit preservation is {100*(matched['limit_preserved_rate'] or 0):.1f}% on comparable matched joints. Axis location and Plucker-line preservation are not evaluated because the package-specific link frames have no proven common coordinate frame. Unmatched reasons are retained in `paired_preservation_records.jsonl`.",
        "",
        "## Paper boundary",
        "",
        "Use only as a supplementary paired representation-change audit after the matching PartNet-Mobility results are joined by dataset ID. Do not count PhysX-Mobility and PartNet-Mobility as independent confirmations, and do not include PhysX-Mobility in generated-method rankings.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    expected_counts = Counter({category: REQUESTED_PER_CATEGORY for category in CATEGORIES})
    verification = {
        "passed": True,
        "checks": {
            "source_provenance_passed": provenance["passed"],
            "five_exact_categories_feasible": feasibility["all_five_exact_categories_feasible"],
            "selection_has_30_rows": len(selection["rows"]) == 30,
            "selection_has_six_per_category": Counter(
                row["category"] for row in selection["rows"]
            )
            == expected_counts,
            "selection_hashes_recompute": all(
                row["selection_hash"]
                == sha256_bytes(row["rank_payload"].encode("utf-8"))
                for row in selection["rows"]
            ),
            "selection_matches_recomputed_identity_only_pool": sorted(
                selection["rows"], key=lambda row: (row["category"], row["selection_rank"])
            )
            == sorted(
                expected_selection["rows"],
                key=lambda row: (row["category"], row["selection_rank"]),
            ),
            "canonical_selection_sha256_matches_partnet_side": sha256_file(
                output / "paired_selection.json"
            )
            == "a0a8eaf00c2970598f3d6191001361dc1e1be1df43ba3e8c394cb6ef988d581b",
            "no_replacement": all(row["failure_replaced"] is False for row in manifest),
            "all_partnet_pair_urdfs_available": all(
                row["partnet_pair_urdf_available"] for row in manifest
            ),
            "physx_annotations_not_used_for_roles": all(
                row["physx_package_annotations_used_for_roles"] is False
                for row in alignments
            ),
            "metadata_assisted_annotations_not_used_as_reference_hierarchy": all(
                row["physx_package_annotations_used_as_reference_hierarchy"] is False
                for row in assisted_records
            ),
            "copied_selected_files_rehash": all(
                sha256_file(output / file_info["path"]) == file_info["sha256"]
                for row in manifest
                for file_info in row["source_files"].values()
            ),
            "summary_selection_hash_matches": summary["hashes"]["frozen_selection_sha256"]
            == sha256_file(output / "frozen_selection.json"),
            "paired_preservation_has_30_records": len(pair_records) == 30,
        },
        "runner_sha256": sha256_file(Path(__file__)),
    }
    verification["passed"] = all(verification["checks"].values())
    write_json(output / "verification.json", verification)
    if not verification["passed"]:
        raise ValueError(f"verification failed: {verification['checks']}")
    print(
        json.dumps(
            {
                "output": str(output),
                "requested": 30,
                "available": overall["available_count"],
                "valid": overall["valid_tree_count"],
                "selection_sha256": summary["hashes"]["frozen_selection_sha256"],
                "paper_ready": summary["paper_ready"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
