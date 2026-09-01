#!/usr/bin/env python3
"""Freeze and evaluate the direct PartNet-Mobility dataset root."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import random
import shutil
from typing import Any, Iterable
import urllib.request
import zipfile

from hierarchy_extended_metrics import aggregate as aggregate_structure
from hierarchy_extended_metrics import analyze_urdf, topology_consistency
from partnet_hierarchy_correctness import aggregate as aggregate_alignment
from partnet_hierarchy_correctness import evaluate_urdf, load_protocol


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_DATASET = WORKSPACE / "PartNet_Mobility/data/dataset"
DEFAULT_ARCHIVE = WORKSPACE / "PartNet_Mobility/partnet-mobility-v0.zip"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference"
DEFAULT_ONTOLOGY_PROTOCOL = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_REFERENCE_PROTOCOL = EXP_ROOT / "reference/partnet_mobility_official_reference_v1.json"
OFFICIAL_API = "https://huggingface.co/api/datasets/sapien-sim/PartNetMobility"
OFFICIAL_REVISION = "ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f"
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
REQUESTED_PER_CATEGORY = 6
EXPECTED_INSTANCE_COUNT = 2347
EXPECTED_ARCHIVE_SHA256 = "b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff"
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260811


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


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


def parse_semantics(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    labels: dict[str, str] = {}
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        fields = line.split(maxsplit=2)
        if len(fields) != 3:
            raise ValueError(f"semantics line {line_number} does not have three fields")
        link_name, joint_kind, raw_part_name = fields
        if link_name in labels:
            raise ValueError(f"duplicate semantics label for {link_name}")
        labels[link_name] = raw_part_name
        rows.append(
            {
                "link_name": link_name,
                "joint_kind": joint_kind,
                "raw_part_name": raw_part_name,
                "source": "semantics.txt",
            }
        )
    return labels, rows


def quantile(values: list[float], probability: float) -> float:
    position = (len(values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    weight = position - lower
    return values[lower] * (1 - weight) + values[upper] * weight


def coverage_weighted_f1(row: dict[str, Any]) -> float:
    return float(row.get("parent_child_edge_f1") or 0.0) * float(
        row.get("semantic_role_coverage") or 0.0
    )


def bootstrap(records: list[dict[str, Any]]) -> dict[str, Any]:
    grouped = {
        category: [row for row in records if row["category"] == category]
        for category in sorted(CATEGORY_MAP.values())
    }
    if any(len(rows) != REQUESTED_PER_CATEGORY for rows in grouped.values()):
        raise ValueError("bootstrap requires six frozen records per category")
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
        "design": "category-stratified asset bootstrap; six draws per category",
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED,
        "ranking_or_pairwise_difference": False,
    }


def aggregate_structure_panel(records: list[dict[str, Any]]) -> dict[str, Any]:
    per_category = {}
    for category in sorted(CATEGORY_MAP.values()):
        rows = [row for row in records if row["category"] == category]
        evaluated = [row for row in rows if row.get("evaluated")]
        per_category[category] = {
            "metrics": aggregate_structure(evaluated, requested_count=len(rows)),
            "topology_consistency": topology_consistency(evaluated),
        }
    fields = ("unique_signature_rate", "mode_rate", "pairwise_exact_rate", "normalized_entropy")
    macro = {}
    for field in fields:
        values = [
            float(row["topology_consistency"][field])
            for row in per_category.values()
            if row["topology_consistency"][field] is not None
        ]
        macro[field] = sum(values) / len(values) if values else None
    evaluated = [row for row in records if row.get("evaluated")]
    return {
        "overall": aggregate_structure(evaluated, requested_count=len(records)),
        "per_category": per_category,
        "category_macro_topology_consistency": macro,
        "pooled_topology_consistency_diagnostic": topology_consistency(evaluated),
        "topology_primary_weighting": "simple mean over five per-category statistics",
    }


def official_identity_audit(selected_ids: list[str]) -> dict[str, Any]:
    result: dict[str, Any] = {"api_url": OFFICIAL_API, "audit_complete": False}
    try:
        with urllib.request.urlopen(OFFICIAL_API, timeout=30) as response:  # noqa: S310
            payload = json.load(response)
        filenames = {str(row["rfilename"]) for row in payload.get("siblings", [])}
        presence = {dataset_id: f"{dataset_id}.zip" in filenames for dataset_id in selected_ids}
        result.update(
            {
                "audit_complete": True,
                "observed_revision": payload.get("sha"),
                "expected_revision": OFFICIAL_REVISION,
                "revision_matches_pin": payload.get("sha") == OFFICIAL_REVISION,
                "gated": payload.get("gated"),
                "declared_license": payload.get("cardData", {}).get("license"),
                "selected_id_zip_presence": presence,
                "all_selected_ids_present": all(presence.values()),
                "selected_local_bytes_authenticated_to_revision": False,
                "authentication_attempt": "hf download of a selected per-ID archive returned access denied for the active account",
            }
        )
    except Exception as exc:  # noqa: BLE001
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def validate_reference_protocol(protocol: dict[str, Any]) -> None:
    if protocol.get("protocol_id") != "partnet_mobility_official_reference_v1":
        raise ValueError("unexpected reference protocol")
    if protocol["selection"]["rank_payload_format"] != "<dataset_id>":
        raise ValueError("reference protocol does not preserve the frozen v0 selection")
    if protocol["selection"]["rank_payload_salt"] is not None:
        raise ValueError("main reference selection must remain unsalted for cohort continuity")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--ontology-protocol", type=Path, default=DEFAULT_ONTOLOGY_PROTOCOL)
    parser.add_argument("--reference-protocol", type=Path, default=DEFAULT_REFERENCE_PROTOCOL)
    parser.add_argument("--selection", type=Path)
    args = parser.parse_args()

    dataset_root = contained(args.dataset_root)
    archive_path = contained(args.archive)
    ontology_path = contained(args.ontology_protocol)
    reference_protocol_path = contained(args.reference_protocol)
    output = contained(args.output, exists=False)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite nonempty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    selected_root = output / "selected_files"
    selected_root.mkdir()
    ontology = load_protocol(ontology_path)
    reference_protocol = json.loads(reference_protocol_path.read_text(encoding="utf-8"))
    validate_reference_protocol(reference_protocol)

    instance_dirs = sorted(path for path in dataset_root.iterdir() if path.is_dir())
    candidates: dict[str, list[dict[str, Any]]] = defaultdict(list)
    identity_parse_failures = []
    for instance in instance_dirs:
        dataset_id = instance.name
        meta_path = instance / "meta.json"
        try:
            metadata = json.loads(meta_path.read_text(encoding="utf-8"))
        except Exception as exc:  # eligibility cannot infer a category without metadata.
            identity_parse_failures.append({"dataset_id": dataset_id, "error": f"{type(exc).__name__}: {exc}"})
            continue
        raw_category = str(metadata.get("model_cat", ""))
        if raw_category not in CATEGORY_MAP:
            continue
        category = CATEGORY_MAP[raw_category]
        candidates[category].append(
            {
                "dataset_id": dataset_id,
                "category": category,
                "raw_category": raw_category,
                "meta_anno_id": metadata.get("anno_id"),
                "model_id": metadata.get("model_id"),
                "selection_hash": sha256_bytes(dataset_id.encode("utf-8")),
            }
        )

    candidate_counts = {category: len(candidates[category]) for category in sorted(candidates)}
    if set(candidate_counts) != set(CATEGORY_MAP.values()):
        raise ValueError("one or more exact target categories are absent")
    if args.selection:
        selection_path = contained(args.selection)
        selection = json.loads(selection_path.read_text(encoding="utf-8"))
        frozen = list(selection["records"])
    else:
        frozen = []
        for category in sorted(CATEGORY_MAP.values()):
            ranked = sorted(candidates[category], key=lambda row: (row["selection_hash"], row["dataset_id"]))
            if len(ranked) < REQUESTED_PER_CATEGORY:
                raise ValueError(f"insufficient candidates for {category}")
            frozen.extend({**row, "selection_rank": rank} for rank, row in enumerate(ranked[:6], 1))
        selection = {
            "protocol_id": reference_protocol["protocol_id"],
            "selection_rule": reference_protocol["selection"],
            "candidate_counts": candidate_counts,
            "records": frozen,
        }
    frozen.sort(key=lambda row: (row["category"], int(row["selection_rank"])))
    if len(frozen) != 30 or Counter(row["category"] for row in frozen) != Counter({c: 6 for c in CATEGORY_MAP.values()}):
        raise ValueError("frozen selection is not five categories x six")
    candidate_index = {(row["category"], row["dataset_id"]): row for rows in candidates.values() for row in rows}
    for row in frozen:
        expected = candidate_index.get((row["category"], row["dataset_id"]))
        if expected is None or row["selection_hash"] != expected["selection_hash"]:
            raise ValueError(f"selection record is not eligible or hash mismatches: {row}")

    selection_path = output / "frozen_selection.json"
    reference_snapshot = output / "reference_protocol_snapshot.json"
    ontology_snapshot = output / "ontology_protocol_snapshot.json"
    write_json(selection_path, selection)
    shutil.copyfile(reference_protocol_path, reference_snapshot)
    shutil.copyfile(ontology_path, ontology_snapshot)

    archive_hash = sha256_file(archive_path)
    if archive_hash != EXPECTED_ARCHIVE_SHA256:
        raise ValueError("companion archive SHA256 does not match the pinned payload")
    manifest = []
    structures = []
    raw_records = []
    semantics_records = []
    root_records = []
    label_records = []
    archive_continuity = []
    with zipfile.ZipFile(archive_path) as source_zip:
        archive_members = set(source_zip.namelist())
        for selected in frozen:
            category = str(selected["category"])
            dataset_id = str(selected["dataset_id"])
            source = dataset_root / dataset_id
            destination = selected_root / category / dataset_id
            destination.mkdir(parents=True)
            files = {
                "meta": source / "meta.json",
                "urdf": source / "mobility.urdf",
                "semantics": source / "semantics.txt",
            }
            missing = [key for key, path in files.items() if not path.is_file()]
            row: dict[str, Any] = {
                "method": "PartNet-Mobility (direct-root real-data reference)",
                **selected,
                "sample_id": f"{category}/{dataset_id}",
                "available": False,
                "missing_required_files": missing,
                "load_error": None,
            }
            try:
                if missing:
                    raise FileNotFoundError(f"missing required files: {missing}")
                extracted = {}
                for key, path in files.items():
                    target = destination / path.name
                    shutil.copyfile(path, target)
                    extracted[key] = {
                        "source_path": str(path),
                        "path": target.relative_to(output).as_posix(),
                        "sha256": sha256_file(target),
                        "size_bytes": target.stat().st_size,
                    }
                    member = f"dataset/{dataset_id}/{path.name}"
                    member_present = member in archive_members
                    member_hash = sha256_bytes(source_zip.read(member)) if member_present else None
                    archive_continuity.append(
                        {
                            "sample_id": row["sample_id"],
                            "file_kind": key,
                            "archive_member": member,
                            "archive_member_present": member_present,
                            "direct_root_sha256": extracted[key]["sha256"],
                            "archive_member_sha256": member_hash,
                            "identical": member_present and member_hash == extracted[key]["sha256"],
                        }
                    )
                row["files"] = extracted
                row["urdf_path"] = extracted["urdf"]["path"]
                row["urdf_sha256"] = extracted["urdf"]["sha256"]
                row["available"] = True
            except Exception as exc:  # frozen failures remain in denominator.
                row["load_error"] = f"{type(exc).__name__}: {exc}"
            manifest.append(row)

            structure = dict(row)
            structure["evaluated"] = False
            if row["available"]:
                try:
                    structure.update(analyze_urdf(output / row["urdf_path"]))
                    structure["evaluated"] = True
                    structure["evaluation_error"] = None
                except Exception as exc:  # noqa: BLE001
                    structure["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            structures.append(structure)

            views = [dict(row), dict(row), dict(row)]
            for view in views:
                view["evaluation_complete"] = False
            if row["available"]:
                urdf = output / row["urdf_path"]
                try:
                    views[0].update(evaluate_urdf(urdf, category, ontology))
                    views[0]["evaluation_complete"] = True
                    views[0]["evaluation_error"] = None
                except Exception as exc:  # noqa: BLE001
                    views[0]["evaluation_error"] = f"{type(exc).__name__}: {exc}"
                try:
                    labels, source_labels = parse_semantics(output / row["files"]["semantics"]["path"])
                    views[1].update(evaluate_urdf(urdf, category, ontology, labels))
                    views[1]["evaluation_complete"] = True
                    views[1]["evaluation_error"] = None
                    views[1]["annotation_label_count"] = len(labels)
                    label_records.append({"sample_id": row["sample_id"], "labels": source_labels})
                    root_labels = dict(labels)
                    root_labels["base"] = BODY_LABEL[category]
                    views[2].update(evaluate_urdf(urdf, category, ontology, root_labels))
                    views[2]["evaluation_complete"] = True
                    views[2]["evaluation_error"] = None
                    views[2]["evaluator_imputed_labels"] = {"base": BODY_LABEL[category]}
                except Exception as exc:  # noqa: BLE001
                    views[1]["evaluation_error"] = f"{type(exc).__name__}: {exc}"
                    views[2]["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            raw_records.append(views[0])
            semantics_records.append(views[1])
            root_records.append(views[2])

    paths = {
        "manifest": output / "manifest.jsonl",
        "structure": output / "structure_records.jsonl",
        "raw": output / "urdf_name_only_records.jsonl",
        "semantics": output / "package_semantics_assisted_records.jsonl",
        "root": output / "root_imputed_sensitivity_records.jsonl",
        "labels": output / "package_semantics_labels.jsonl",
        "archive_continuity": output / "archive_continuity_records.jsonl",
    }
    write_jsonl(paths["manifest"], manifest)
    write_jsonl(paths["structure"], structures)
    write_jsonl(paths["raw"], raw_records)
    write_jsonl(paths["semantics"], semantics_records)
    write_jsonl(paths["root"], root_records)
    write_jsonl(paths["labels"], label_records)
    write_jsonl(paths["archive_continuity"], archive_continuity)

    official_audit = official_identity_audit([row["dataset_id"] for row in frozen])
    old_manifest_path = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/partnet_mobility_reference/manifest.jsonl"
    old_manifest = [json.loads(line) for line in old_manifest_path.read_text(encoding="utf-8").splitlines()]
    old_ids = [row["dataset_id"] for row in old_manifest]
    new_ids = [row["dataset_id"] for row in manifest]
    old_hashes = {row["sample_id"]: row["urdf_sha256"] for row in old_manifest}
    new_hashes = {row["sample_id"]: row.get("urdf_sha256") for row in manifest}
    continuity = {
        "old_reference_manifest": str(old_manifest_path),
        "same_ordered_selected_ids": old_ids == new_ids,
        "same_selected_id_set": set(old_ids) == set(new_ids),
        "all_selected_urdf_hashes_identical": old_hashes == new_hashes,
        "direct_root_vs_companion_archive_file_count": len(archive_continuity),
        "all_direct_root_selected_files_identical_to_archive_members": all(row["identical"] for row in archive_continuity),
    }
    provenance = {
        "dataset_root": {"path": str(dataset_root), "instance_directory_count": len(instance_dirs)},
        "companion_archive": {"path": str(archive_path), "sha256": archive_hash, "size_bytes": archive_path.stat().st_size},
        "official_repository": official_audit,
        "license": {
            "declared_hub_value": official_audit.get("declared_license"),
            "terms": "non-commercial research and educational use; ShapeNet terms also apply",
            "source": "official sapien-sim/PartNetMobility dataset card",
        },
        "local_payload_continuity": continuity,
        "provenance_status": "PROVENANCE_LIMITED",
        "provenance_conclusion": "The direct root is byte-identical to the previously evaluated local archive for all 30 selected meta/URDF/semantics files, and every selected ID is listed by the pinned official repository. The selected local bytes were not directly authenticated against official-revision per-ID archives.",
        "paper_ready": False,
        "paper_ready_blockers": [
            "selected local bytes are not directly authenticated to the pinned official repository revision",
            "annotation-assisted views share ontology provenance with the PartNet-derived evaluator",
        ],
    }
    provenance_path = output / "provenance.json"
    write_json(provenance_path, provenance)

    structure_summary = aggregate_structure_panel(structures)
    raw_summary = aggregate_alignment(raw_records)
    semantics_summary = aggregate_alignment(semantics_records)
    root_summary = aggregate_alignment(root_records)
    semantics_bootstrap = bootstrap(semantics_records)
    summary = {
        "protocol_id": reference_protocol["protocol_id"],
        "display_name": "PartNet-Mobility (direct-root real-data reference; provenance-limited)",
        "role": "real-data reference, not a generation method and excluded from generated-method rankings",
        "requested_count": 30,
        "requested_per_category": 6,
        "candidate_counts": candidate_counts,
        "selection_rule": "within exact package category, sort by SHA256(dataset_id UTF-8), tie-break by dataset_id, take six; retain failures without replacement",
        "structure": structure_summary,
        "urdf_name_only_sensitivity": raw_summary,
        "package_semantics_assisted_calibration": semantics_summary,
        "package_semantics_assisted_bootstrap": semantics_bootstrap,
        "evaluator_imputed_root_sensitivity": root_summary,
        "same_ontology_provenance_warning": reference_protocol["evaluation"]["same_ontology_provenance_warning"],
        "provenance_status": provenance["provenance_status"],
        "paper_ready": False,
    }
    summary_path = output / "summary.json"
    write_json(summary_path, summary)
    summary["hashes"] = {
        **{f"{key}_sha256": sha256_file(path) for key, path in paths.items()},
        "frozen_selection_sha256": sha256_file(selection_path),
        "reference_protocol_snapshot_sha256": sha256_file(reference_snapshot),
        "ontology_protocol_snapshot_sha256": sha256_file(ontology_snapshot),
        "provenance_sha256": sha256_file(provenance_path),
        "runner_sha256": sha256_file(Path(__file__)),
    }
    write_json(summary_path, summary)

    structure_overall = structure_summary["overall"]
    ci = semantics_bootstrap["ci95_percentile"]
    report = [
        "# PartNet-Mobility direct-root real-data reference",
        "",
        "This is a curated real-data reference, not a generated-method baseline or independent semantic gold.",
        "",
        "## Frozen panel",
        "",
        f"- Requested: 30; available: {sum(bool(row['available']) for row in manifest)}/30; valid trees: {structure_overall['valid_tree_count']}/30.",
        "- Five exact categories x six; identity-only SHA256(dataset_id), preserving the prior frozen cohort; failures are never replaced.",
        f"- Mean nodes/depth/movable joints on valid trees: {structure_overall['node_count_mean']:.3f}/{structure_overall['semantic_depth_mean']:.3f}/{structure_overall['movable_edge_count_mean']:.3f}.",
        "",
        "## PartNet-ontology alignment",
        "",
        "| View | Role coverage | Scorable | Coverage-weighted induced Edge F1 | Semantic-parent alignment |",
        "|---|---:|---:|---:|---:|",
        f"| Raw URDF link names | {100*raw_summary['semantic_role_coverage_requested_macro']:.1f}% | {raw_summary['scorable_count']}/30 | {100*raw_summary['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% | {100*raw_summary['semantic_nesting_accuracy_requested_macro']:.1f}% |",
        f"| Package `semantics.txt` assisted | {100*semantics_summary['semantic_role_coverage_requested_macro']:.1f}% | {semantics_summary['scorable_count']}/30 | {100*semantics_summary['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% [{100*ci[0]:.1f}, {100*ci[1]:.1f}] | {100*semantics_summary['semantic_nesting_accuracy_requested_macro']:.1f}% |",
        f"| Evaluator-imputed category root sensitivity | {100*root_summary['semantic_role_coverage_requested_macro']:.1f}% | {root_summary['scorable_count']}/30 | {100*root_summary['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% | {100*root_summary['semantic_nesting_accuracy_requested_macro']:.1f}% |",
        "",
        "Package semantics share provenance with the PartNet-derived ontology. The root-imputed result additionally modifies labels. Neither is independent hierarchy correctness evidence.",
        "",
        "## Provenance",
        "",
        f"- Frozen cohort identical to old reference: {continuity['same_ordered_selected_ids']}; selected meta/URDF/semantics files identical to companion archive: {continuity['all_direct_root_selected_files_identical_to_archive_members']}.",
        f"- Official pinned revision lists every selected ID: {official_audit.get('all_selected_ids_present')}; local byte authentication to that revision: false.",
        "- Status: `PROVENANCE_LIMITED`; this run does not remove the old source-authentication blocker.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    checks = {
        "instance_directory_count_is_2347": len(instance_dirs) == EXPECTED_INSTANCE_COUNT,
        "identity_metadata_parse_failures_zero": not identity_parse_failures,
        "manifest_has_30_rows": len(manifest) == 30,
        "six_per_category": Counter(row["category"] for row in manifest) == Counter({c: 6 for c in CATEGORY_MAP.values()}),
        "selection_hashes_recompute": all(row["selection_hash"] == sha256_bytes(row["dataset_id"].encode("utf-8")) for row in manifest),
        "selection_ranks_are_1_to_6": all(sorted(row["selection_rank"] for row in manifest if row["category"] == c) == list(range(1, 7)) for c in CATEGORY_MAP.values()),
        "frozen_failures_not_replaced": len({row["sample_id"] for row in manifest}) == 30,
        "all_selected_files_recompute": all(all(sha256_file(output / file_row["path"]) == file_row["sha256"] for file_row in row.get("files", {}).values()) for row in manifest),
        "companion_archive_hash_matches": archive_hash == EXPECTED_ARCHIVE_SHA256,
        "all_selected_files_match_companion_archive": continuity["all_direct_root_selected_files_identical_to_archive_members"],
        "same_ordered_selection_as_old_reference": continuity["same_ordered_selected_ids"],
        "same_selected_urdf_hashes_as_old_reference": continuity["all_selected_urdf_hashes_identical"],
        "official_revision_matches_pin": official_audit.get("revision_matches_pin") is True,
        "all_selected_ids_listed_officially": official_audit.get("all_selected_ids_present") is True,
        "selected_bytes_not_misrepresented_as_officially_authenticated": official_audit.get("selected_local_bytes_authenticated_to_revision") is False,
    }
    verification = {"passed": all(checks.values()), "checks": checks, "summary_sha256": sha256_file(summary_path), "runner_sha256": sha256_file(Path(__file__))}
    write_json(output / "verification.json", verification)
    if not verification["passed"]:
        raise ValueError(f"verification failed: {checks}")
    print(json.dumps({"output": str(output), "valid": structure_overall["valid_tree_count"], "paper_ready": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
