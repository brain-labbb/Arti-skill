#!/usr/bin/env python3
"""Build a provenance-limited PartNet-Mobility real-data reference panel."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import random
import urllib.request
from typing import Any
import zipfile

from hierarchy_extended_metrics import (
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
DEFAULT_ARCHIVE = WORKSPACE / "PartNet_Mobility/partnet-mobility-v0.zip"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/partnet_mobility_reference"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
EXPECTED_ARCHIVE_SHA256 = "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
OFFICIAL_REPO_API = "https://huggingface.co/api/datasets/sapien-sim/PartNetMobility"
OFFICIAL_REPO_REVISION_AT_AUDIT = "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f"
HF_POINT_CLOUD_REPO = "yuchen0187/partnet-mobility"
HF_POINT_CLOUD_REVISION = "bf39e304f19a6c131b5244f128b79ec35000bb02"
CATEGORY_MAP = {
    "StorageFurniture": "storage_furniture",
    "Table": "table",
    "Refrigerator": "refrigerator",
    "Dishwasher": "dishwasher",
    "Microwave": "microwave",
}
BODY_LABEL = {
    "storage_furniture": "furniture_body",
    "table": "furniture_body",
    "refrigerator": "refrigerator_body",
    "dishwasher": "dishwasher_body",
    "microwave": "microwave_body",
}
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260811


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


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def quantile(sorted_values: list[float], probability: float) -> float:
    position = (len(sorted_values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def coverage_weighted_f1(row: dict[str, Any]) -> float:
    return float(row.get("parent_child_edge_f1") or 0.0) * float(
        row.get("semantic_role_coverage") or 0.0
    )


def bootstrap_primary(records: list[dict[str, Any]]) -> dict[str, Any]:
    grouped = {
        category: [row for row in records if row["category"] == category]
        for category in sorted(CATEGORY_MAP.values())
    }
    if any(len(rows) != 6 for rows in grouped.values()):
        raise ValueError("bootstrap requires exactly six frozen records per category")
    estimate = sum(coverage_weighted_f1(row) for row in records) / len(records)
    rng = random.Random(BOOTSTRAP_SEED)
    samples = []
    for _ in range(BOOTSTRAP_REPLICATES):
        sampled = [rng.choice(rows) for rows in grouped.values() for _ in range(6)]
        samples.append(sum(coverage_weighted_f1(row) for row in sampled) / len(sampled))
    samples.sort()
    return {
        "metric": "coverage_weighted_induced_edge_f1_requested_macro",
        "estimate": estimate,
        "ci95_percentile": [quantile(samples, 0.025), quantile(samples, 0.975)],
        "design": "category-stratified asset bootstrap with replacement; six draws within each of five categories",
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "ranking_or_pairwise_difference": False,
    }


def parse_semantics(text: str) -> tuple[dict[str, str], list[dict[str, str]]]:
    labels: dict[str, str] = {}
    records: list[dict[str, str]] = []
    for line_number, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if not stripped:
            continue
        fields = stripped.split(maxsplit=2)
        if len(fields) != 3:
            raise ValueError(f"semantics.txt line {line_number} does not have 3 fields")
        link_name, joint_kind, part_name = fields
        if link_name in labels:
            raise ValueError(f"duplicate semantic label for {link_name}")
        labels[link_name] = part_name
        records.append(
            {
                "link_name": link_name,
                "joint_kind": joint_kind,
                "raw_part_name": part_name,
                "source": "semantics.txt",
            }
        )
    return labels, records


def official_identity_audit(selected_ids: list[str]) -> dict[str, Any]:
    result: dict[str, Any] = {
        "api_url": OFFICIAL_REPO_API,
        "audit_complete": False,
        "selected_id_count": len(selected_ids),
    }
    try:
        with urllib.request.urlopen(OFFICIAL_REPO_API, timeout=30) as response:  # noqa: S310
            payload = json.load(response)
        filenames = {str(row["rfilename"]) for row in payload.get("siblings", [])}
        present = {model_id: f"{model_id}.zip" in filenames for model_id in selected_ids}
        result.update(
            {
                "audit_complete": True,
                "expected_repo_revision": OFFICIAL_REPO_REVISION_AT_AUDIT,
                "observed_repo_revision": payload.get("sha"),
                "revision_matches_audit_pin": payload.get("sha")
                == OFFICIAL_REPO_REVISION_AT_AUDIT,
                "gated": payload.get("gated"),
                "declared_license": payload.get("cardData", {}).get("license"),
                "selected_id_zip_presence": present,
                "all_selected_ids_present": all(present.values()),
                "interpretation": (
                    "Filename presence establishes that selected IDs are listed by the official "
                    "repository. It does not authenticate the bytes in the local source archive."
                ),
            }
        )
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    args = parser.parse_args()

    archive_path = contained(args.archive)
    protocol_path = contained(args.protocol)
    output = contained(args.output, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    selected_root = output / "selected_files"
    selected_root.mkdir(parents=True, exist_ok=True)

    archive_hash = sha256_file(archive_path)
    if archive_hash != EXPECTED_ARCHIVE_SHA256:
        raise ValueError(f"unexpected source archive SHA-256: {archive_hash}")
    protocol = load_protocol(protocol_path)

    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    archive_member_count = 0
    with zipfile.ZipFile(archive_path) as source_zip:
        members = set(source_zip.namelist())
        archive_member_count = len(members)
        for name in sorted(members):
            fields = name.split("/")
            if len(fields) != 3 or fields[0] != "dataset" or fields[2] != "meta.json":
                continue
            dataset_id = fields[1]
            try:
                metadata = json.loads(source_zip.read(name))
            except Exception:  # Selection never filters on later parse/load viability.
                continue
            raw_category = str(metadata.get("model_cat", ""))
            if raw_category not in CATEGORY_MAP:
                continue
            category = CATEGORY_MAP[raw_category]
            candidates[category].append(
                {
                    "dataset_id": dataset_id,
                    "meta_anno_id": metadata.get("anno_id"),
                    "model_id": metadata.get("model_id"),
                    "raw_category": raw_category,
                    "category": category,
                    "meta_member": name,
                    "selection_hash": sha256_bytes(dataset_id.encode("utf-8")),
                }
            )

        expected_categories = set(CATEGORY_MAP.values())
        if set(candidates) != expected_categories:
            raise ValueError(f"missing source categories: {sorted(expected_categories - set(candidates))}")

        frozen: list[dict[str, Any]] = []
        candidate_counts = {}
        for category in sorted(candidates):
            ranked = sorted(
                candidates[category], key=lambda row: (row["selection_hash"], row["dataset_id"])
            )
            candidate_counts[category] = len(ranked)
            for rank, row in enumerate(ranked[:6], 1):
                frozen.append({**row, "selection_rank": rank})
        frozen.sort(key=lambda row: (row["category"], row["selection_rank"]))

        manifest = []
        structure_records = []
        name_only_records = []
        annotation_records = []
        imputed_root_records = []
        annotation_label_records = []
        for selected in frozen:
            category = str(selected["category"])
            dataset_id = str(selected["dataset_id"])
            prefix = f"dataset/{dataset_id}"
            required_members = {
                "meta": f"{prefix}/meta.json",
                "urdf": f"{prefix}/mobility.urdf",
                "semantics": f"{prefix}/semantics.txt",
            }
            missing = [key for key, member in required_members.items() if member not in members]
            destination = selected_root / category / dataset_id
            destination.mkdir(parents=True, exist_ok=True)
            row: dict[str, Any] = {
                "method": "PartNet-Mobility (real-data reference)",
                **selected,
                "sample_id": f"{category}/{dataset_id}",
                "available": False,
                "missing_required_members": missing,
                "load_error": None,
                "official_authentication": False,
            }
            try:
                if missing:
                    raise FileNotFoundError(f"missing archive members: {missing}")
                extracted = {}
                for key, member in required_members.items():
                    data = source_zip.read(member)
                    suffix = {"meta": "meta.json", "urdf": "mobility.urdf", "semantics": "semantics.txt"}[key]
                    path = destination / suffix
                    path.write_bytes(data)
                    extracted[key] = {
                        "path": path.relative_to(output).as_posix(),
                        "sha256": sha256_bytes(data),
                        "size_bytes": len(data),
                    }
                row["files"] = extracted
                row["urdf_path"] = extracted["urdf"]["path"]
                row["urdf_sha256"] = extracted["urdf"]["sha256"]
                row["available"] = True
            except Exception as exc:  # Frozen failure remains in denominator; no replacement.
                row["load_error"] = f"{type(exc).__name__}: {exc}"
            manifest.append(row)

            structure_row = dict(row)
            structure_row["evaluated"] = False
            if row["available"]:
                try:
                    structure_row.update(analyze_urdf(output / row["urdf_path"]))
                    structure_row["evaluated"] = True
                    structure_row["evaluation_error"] = None
                except Exception as exc:  # noqa: BLE001
                    structure_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            structure_records.append(structure_row)

            name_row = dict(row)
            name_row["evaluation_complete"] = False
            annotation_row = dict(row)
            annotation_row["evaluation_complete"] = False
            imputed_row = dict(row)
            imputed_row["evaluation_complete"] = False
            if row["available"]:
                urdf_path = output / row["urdf_path"]
                try:
                    name_row.update(evaluate_urdf(urdf_path, category, protocol))
                    name_row["evaluation_complete"] = True
                    name_row["evaluation_error"] = None
                except Exception as exc:  # noqa: BLE001
                    name_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
                try:
                    semantics_path = output / row["files"]["semantics"]["path"]
                    labels, label_records = parse_semantics(
                        semantics_path.read_text(encoding="utf-8")
                    )
                    annotation_row.update(evaluate_urdf(urdf_path, category, protocol, labels))
                    annotation_row["evaluation_complete"] = True
                    annotation_row["evaluation_error"] = None
                    annotation_row["annotation_label_count"] = len(labels)
                    annotation_label_records.append(
                        {
                            "sample_id": row["sample_id"],
                            "assignment_status": "PACKAGE_ANNOTATION_ASSISTED_CALIBRATION_NOT_METHOD_RANKING",
                            "labels": label_records,
                        }
                    )
                except Exception as exc:  # noqa: BLE001
                    annotation_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
                try:
                    labels_with_root = dict(labels)
                    labels_with_root["base"] = BODY_LABEL[category]
                    imputed_row.update(
                        evaluate_urdf(urdf_path, category, protocol, labels_with_root)
                    )
                    imputed_row["evaluation_complete"] = True
                    imputed_row["evaluation_error"] = None
                    imputed_row["annotation_label_count"] = len(labels)
                    imputed_row["evaluator_imputed_label_count"] = 1
                    imputed_row["evaluator_imputed_labels"] = {
                        "base": BODY_LABEL[category]
                    }
                except Exception as exc:  # noqa: BLE001
                    imputed_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            name_only_records.append(name_row)
            annotation_records.append(annotation_row)
            imputed_root_records.append(imputed_row)

    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in structure_records:
        by_category[str(row["category"])].append(row)
    per_category_structure = {}
    for category, rows in sorted(by_category.items()):
        evaluated = [row for row in rows if row.get("evaluated")]
        per_category_structure[category] = {
            "metrics": aggregate_structure(evaluated, requested_count=len(rows)),
            "topology_consistency": topology_consistency(evaluated),
        }
    evaluated_structure = [row for row in structure_records if row.get("evaluated")]
    topology_fields = (
        "unique_signature_rate",
        "mode_rate",
        "pairwise_exact_rate",
        "normalized_entropy",
    )
    category_macro_topology = {}
    for field in topology_fields:
        values = [
            float(item["topology_consistency"][field])
            for item in per_category_structure.values()
            if item["topology_consistency"][field] is not None
        ]
        category_macro_topology[field] = sum(values) / len(values) if values else None
    structure_summary = {
        "overall": aggregate_structure(evaluated_structure, requested_count=len(structure_records)),
        "per_category": per_category_structure,
        "category_macro_topology_consistency": category_macro_topology,
        "pooled_topology_consistency_diagnostic": topology_consistency(evaluated_structure),
        "topology_primary_weighting": "simple mean over five per-category topology statistics",
    }
    name_summary = aggregate_alignment(name_only_records)
    annotation_summary = aggregate_alignment(annotation_records)
    imputed_root_summary = aggregate_alignment(imputed_root_records)
    annotation_bootstrap = bootstrap_primary(annotation_records)
    selected_ids = [str(row["dataset_id"]) for row in manifest]
    official_audit = official_identity_audit(selected_ids)

    manifest_path = output / "manifest.jsonl"
    structure_path = output / "structure_records.jsonl"
    name_path = output / "urdf_name_only_records.jsonl"
    annotation_path = output / "annotation_assisted_records.jsonl"
    imputed_root_path = output / "evaluator_imputed_root_records.jsonl"
    labels_path = output / "annotation_labels.jsonl"
    write_jsonl(manifest_path, manifest)
    write_jsonl(structure_path, structure_records)
    write_jsonl(name_path, name_only_records)
    write_jsonl(annotation_path, annotation_records)
    write_jsonl(imputed_root_path, imputed_root_records)
    write_jsonl(labels_path, annotation_label_records)

    provenance = {
        "source_archive": {
            "path": str(archive_path),
            "size_bytes": archive_path.stat().st_size,
            "sha256": archive_hash,
            "member_count": archive_member_count,
            "official_authentication": False,
            "provenance_status": "local archive filename and contents are consistent with PartNet-Mobility v0, but byte provenance is not authenticated",
            "declared_license_in_archive": None,
        },
        "official_repository_identity_audit": official_audit,
        "requested_hf_mirror": {
            "repo": HF_POINT_CLOUD_REPO,
            "revision": HF_POINT_CLOUD_REVISION,
            "declared_license": None,
            "schema": ["xyz", "rgb", "mask"],
            "usable_as_hierarchy_reference": False,
            "reason": "No category, object identity, URDF, semantic hierarchy, or mobility annotation fields are published.",
        },
        "vlongle_preprocessed_mirror": {
            "repo": "vlongle/articulate-anything-dataset-preprocessed",
            "revision": "b07e6a2b979c67d5782a9b12cc2625de7b04386b",
            "declared_license": None,
            "used": False,
        },
        "protocol": {
            "path": str(protocol_path),
            "sha256": sha256_file(protocol_path),
            "same_ontology_provenance_warning": (
                "The annotation-assisted calibration uses only PartNet-Mobility semantics.txt labels and a PartNet-derived category ontology. "
                "This is in-domain and potentially circular, so it is excluded from generated-method rankings."
            ),
        },
        "paper_ready": False,
        "paper_ready_blockers": [
            "local source archive bytes are not authenticated to an official revision",
            "the local archive does not declare a license",
            "annotation-assisted alignment is ontology-provenance-coupled",
        ],
    }
    provenance_path = output / "provenance.json"
    write_json(provenance_path, provenance)

    summary = {
        "protocol_id": "partnet_mobility_real_data_reference_v1",
        "display_name": "PartNet-Mobility (real-data reference; provenance-limited)",
        "role": "real-data reference, not a generation method and not included in generated-method rankings",
        "requested_count": 30,
        "requested_per_category": 6,
        "categories": sorted(CATEGORY_MAP.values()),
        "candidate_counts": candidate_counts,
        "selection_rule": "within each package category, sort by SHA256(dataset directory ID UTF-8), tie-break by dataset_id, take first six; retain failures without replacement",
        "structure": structure_summary,
        "urdf_name_only_sensitivity": name_summary,
        "package_annotation_assisted_calibration": annotation_summary,
        "package_annotation_assisted_primary_bootstrap": annotation_bootstrap,
        "evaluator_imputed_root_sensitivity": imputed_root_summary,
        "same_ontology_provenance_warning": provenance["protocol"]["same_ontology_provenance_warning"],
        "paper_ready": False,
        "hashes": {
            "manifest_sha256": sha256_file(manifest_path),
            "structure_records_sha256": sha256_file(structure_path),
            "urdf_name_only_records_sha256": sha256_file(name_path),
            "annotation_assisted_records_sha256": sha256_file(annotation_path),
            "evaluator_imputed_root_records_sha256": sha256_file(imputed_root_path),
            "annotation_labels_sha256": sha256_file(labels_path),
            "provenance_sha256": sha256_file(provenance_path),
            "runner_sha256": sha256_file(Path(__file__)),
        },
    }
    summary_path = output / "summary.json"
    write_json(summary_path, summary)

    available = sum(bool(row["available"]) for row in manifest)
    structure_overall = structure_summary["overall"]
    topology_macro = structure_summary["category_macro_topology_consistency"]
    report = [
        "# PartNet-Mobility real-data reference (provenance-limited)",
        "",
        "This is a curated real-data reference, not a generation method. It must be excluded from generated-method rankings.",
        "",
        "## Frozen panel",
        "",
        f"- Requested: 30 (five categories x six); available: {available}/30.",
        "- Selection: identity-only SHA256(dataset directory ID), first six per category; failures retained without replacement.",
        f"- Valid URDF trees: {structure_overall['valid_tree_count']}/30.",
        f"- Mean nodes per valid tree: {structure_overall['node_count_mean']:.2f}.",
        f"- Mean movable edges per valid tree: {structure_overall['movable_edge_count_mean']:.2f}.",
        (
            "- Category-macro topology: "
            f"unique-signature rate {100 * topology_macro['unique_signature_rate']:.1f}%, "
            f"mode rate {100 * topology_macro['mode_rate']:.1f}%, "
            f"pairwise exact rate {100 * topology_macro['pairwise_exact_rate']:.1f}%, "
            f"normalized entropy {topology_macro['normalized_entropy']:.3f}."
        ),
        "- Pooled 30-asset topology statistics are retained in `summary.json` as diagnostics only.",
        "",
        "## Semantic alignment",
        "",
        "| View | Available | Role coverage requested | Scorable requested | Coverage-weighted induced Edge F1 requested (95% CI where reported) | Semantic-parent alignment requested |",
        "|---|---:|---:|---:|---:|---:|",
        (
            f"| URDF link names only | {name_summary['available_count']}/30 | "
            f"{100 * name_summary['semantic_role_coverage_requested_macro']:.1f}% | "
            f"{100 * name_summary['scorable_asset_coverage_requested']:.1f}% | "
            f"{100 * name_summary['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% | "
            f"{100 * name_summary['semantic_nesting_accuracy_requested_macro']:.1f}% |"
        ),
        (
            f"| Package annotation assisted | {annotation_summary['available_count']}/30 | "
            f"{100 * annotation_summary['semantic_role_coverage_requested_macro']:.1f}% | "
            f"{100 * annotation_summary['scorable_asset_coverage_requested']:.1f}% | "
            f"{100 * annotation_summary['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% "
            f"[{100 * annotation_bootstrap['ci95_percentile'][0]:.1f}, "
            f"{100 * annotation_bootstrap['ci95_percentile'][1]:.1f}] | "
            f"{100 * annotation_summary['semantic_nesting_accuracy_requested_macro']:.1f}% |"
        ),
        (
            f"| Evaluator-imputed root sensitivity | {imputed_root_summary['available_count']}/30 | "
            f"{100 * imputed_root_summary['semantic_role_coverage_requested_macro']:.1f}% | "
            f"{100 * imputed_root_summary['scorable_asset_coverage_requested']:.1f}% | "
            f"{100 * imputed_root_summary['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% | "
            f"{100 * imputed_root_summary['semantic_nesting_accuracy_requested_macro']:.1f}% |"
        ),
        "",
        "The package annotation-assisted view uses only published `semantics.txt` labels. The separate root sensitivity adds "
        "one evaluator-imputed category-conditioned body label to URDF `base`; it is not package annotation. Both views "
        "share provenance with the PartNet-derived ontology and are calibration only, not ranking evidence.",
        "The primary semantics-only interval uses 10,000 category-stratified bootstrap replicates (seed 20260811), "
        "drawing six assets with replacement within each of five categories. No ranking or pairwise difference is computed.",
        "",
        "## Provenance boundary",
        "",
        f"- Local archive SHA-256: `{archive_hash}`.",
        f"- Selected IDs listed by official gated repository: {official_audit.get('all_selected_ids_present')}.",
        "- Official byte authentication: false. Archive license declaration: absent.",
        "- Paper-ready status: false until source authenticity and license are resolved.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    verification = {
        "passed": True,
        "checks": {
            "manifest_has_30_rows": len(manifest) == 30,
            "six_per_category": Counter(row["category"] for row in manifest)
            == Counter({category: 6 for category in CATEGORY_MAP.values()}),
            "selection_ranks_are_1_to_6": all(
                sorted(row["selection_rank"] for row in manifest if row["category"] == category)
                == list(range(1, 7))
                for category in CATEGORY_MAP.values()
            ),
            "selection_hashes_recompute": all(
                row["selection_hash"] == sha256_bytes(row["dataset_id"].encode("utf-8"))
                for row in manifest
            ),
            "official_revision_matches_audit_pin": official_audit.get(
                "revision_matches_audit_pin"
            )
            is True,
            "no_replacement": len({row["sample_id"] for row in manifest}) == 30,
            "all_extracted_hashes_recompute": all(
                all(
                    sha256_file(output / file_row["path"]) == file_row["sha256"]
                    for file_row in row.get("files", {}).values()
                )
                for row in manifest
            ),
            "summary_hash_matches": sha256_file(manifest_path) == summary["hashes"]["manifest_sha256"],
        },
        "source_archive_sha256": archive_hash,
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
                "requested": len(manifest),
                "available": available,
                "valid_trees": structure_overall["valid_tree_count"],
                "paper_ready": False,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
