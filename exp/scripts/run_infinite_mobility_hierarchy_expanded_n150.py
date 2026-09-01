#!/usr/bin/env python3
"""Evaluate the pre-frozen Infinite Mobility seeds 0-29 on five Table 3 categories."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import hierarchy_extended_metrics as shared
import run_infinite_mobility_baseline as baseline
import run_infinite_mobility_hierarchy as hierarchy
from partnet_hierarchy_correctness import aggregate as aggregate_alignment
from partnet_hierarchy_correctness import evaluate_urdf, load_protocol


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_PROTOCOL = EXP_ROOT / "reference/infinite_mobility_hierarchy_expanded_n150_v1.json"
DEFAULT_SOURCE_PROTOCOL = EXP_ROOT / "reference/infinite_mobility_protocol_v1.json"
DEFAULT_ONTOLOGY = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_INPUT = EXP_ROOT / "runtime/infinite_mobility_v1"
DEFAULT_OLD_PANEL = EXP_ROOT / "runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/infinite_mobility"
EXPECTED_CATEGORIES = ("storage_furniture", "table", "refrigerator", "dishwasher", "microwave")
EXPECTED_SEEDS = tuple(range(30))
NESTED_SEEDS = tuple(range(6))


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8")


def normalize_role(part_name: str) -> str:
    role = part_name.strip().lower()
    if role.endswith("_part"):
        role = role[:-5]
    return {"microwave_body": "body", "botton": "button"}.get(role, role)


def flatten_parts(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, list):
        raise ValueError("data_infos root must be a list")
    rows = []
    for group in payload:
        if not isinstance(group, dict) or not isinstance(group.get("part"), list):
            raise ValueError("data_infos group must contain a part list")
        rows.extend(group["part"])
    return rows


def recover_link_labels(urdf_path: Path, package_root: Path) -> tuple[dict[str, str], dict[str, Any], list[dict[str, Any]]]:
    metadata_paths = sorted(package_root.glob("data_infos_*.json"))
    if len(metadata_paths) != 1:
        raise ValueError(f"expected one data_infos file, found {len(metadata_paths)}")
    metadata_path = metadata_paths[0]
    parts = flatten_parts(json.loads(metadata_path.read_text(encoding="utf-8")))
    counts = Counter(str(row.get("file_name", "")) for row in parts)
    duplicate_filenames = sorted(name for name, count in counts.items() if name and count > 1)
    by_filename = {str(row["file_name"]): row for row in parts if row.get("file_name") and counts[str(row["file_name"])] == 1}
    root = ET.parse(urdf_path).getroot()
    labels: dict[str, str] = {}
    assignments = []
    visual_mesh_count = 0
    missing = []
    conflicting = []
    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        rows_for_link = []
        for mesh in link.findall("visual/geometry/mesh"):
            visual_mesh_count += 1
            mesh_name = Path(mesh.attrib.get("filename", "")).name
            metadata = by_filename.get(mesh_name)
            if metadata is None:
                missing.append({"link_name": link_name, "mesh_filename": mesh_name})
                continue
            raw_role = str(metadata.get("part_name", ""))
            normalized = normalize_role(raw_role)
            rows_for_link.append((raw_role, normalized, mesh_name))
            assignments.append(
                {
                    "link_name": link_name,
                    "mesh_filename": mesh_name,
                    "raw_part_name": raw_role,
                    "predicted_role": normalized,
                    "assignment_status": "PREDICTION_ONLY_NOT_GOLD",
                }
            )
        normalized_roles = {row[1] for row in rows_for_link}
        if len(normalized_roles) > 1:
            conflicting.append({"link_name": link_name, "roles": sorted(normalized_roles)})
        elif normalized_roles:
            labels[link_name] = next(iter(normalized_roles))
    audit = {
        "metadata_path": metadata_path.relative_to(package_root).as_posix(),
        "metadata_sha256": sha256_file(metadata_path),
        "metadata_row_count": len(parts),
        "visual_mesh_count": visual_mesh_count,
        "mapped_visual_mesh_count": len(assignments),
        "mapping_coverage": len(assignments) / visual_mesh_count if visual_mesh_count else None,
        "duplicate_metadata_filenames": duplicate_filenames,
        "missing_metadata_matches": missing,
        "conflicting_roles_within_link": conflicting,
        "link_label_count": len(labels),
        "recoverable": bool(visual_mesh_count and not duplicate_filenames and not missing and not conflicting and len(assignments) == visual_mesh_count),
        "gold_eligible": False,
    }
    return labels, audit, assignments


def category_macro_topology(structure_records: list[dict[str, Any]]) -> tuple[dict[str, Any], dict[str, Any]]:
    per_category = {}
    for category in EXPECTED_CATEGORIES:
        requested = [row for row in structure_records if row["category"] == category]
        evaluated = [row for row in requested if row.get("evaluated")]
        per_category[category] = {
            "aggregate": shared.aggregate(evaluated, requested_count=len(requested)),
            "topology_consistency": shared.topology_consistency(evaluated),
        }
    fields = ("unique_signature_rate", "mode_rate", "pairwise_exact_rate", "normalized_entropy")
    macro = {}
    for field in fields:
        values = [float(row["topology_consistency"][field]) for row in per_category.values() if row["topology_consistency"][field] is not None]
        macro[field] = sum(values) / len(values) if values else None
    return per_category, macro


def load_old_nested(old_panel: Path) -> dict[tuple[str, int], dict[str, Any]]:
    manifest = json.loads((old_panel / "cohort_manifest.json").read_text(encoding="utf-8"))
    return {
        (str(row["ontology_category"]), int(row["seed"])): row
        for row in manifest["selection"]
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--source-protocol", type=Path, default=DEFAULT_SOURCE_PROTOCOL)
    parser.add_argument("--ontology", type=Path, default=DEFAULT_ONTOLOGY)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--old-panel", type=Path, default=DEFAULT_OLD_PANEL)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--selection", type=Path)
    args = parser.parse_args()
    protocol_path = contained(args.protocol)
    source_protocol_path = contained(args.source_protocol)
    ontology_path = contained(args.ontology)
    input_root = contained(args.input_root)
    old_panel = contained(args.old_panel)
    output = contained(args.output_root, exists=False)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite nonempty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    ontology = load_protocol(ontology_path)
    source_protocol, source_manifest, source_records = hierarchy.load_and_validate_inputs(source_protocol_path, input_root)
    if protocol["source_seed_protocol"] != source_protocol["protocol_id"]:
        raise ValueError("source seed protocol mismatch")
    if tuple(protocol["seeds"]) != EXPECTED_SEEDS or tuple(protocol["nested_original_seeds"]) != NESTED_SEEDS:
        raise ValueError("expanded seed selection differs from the supported contract")
    category_by_factory = {row["factory"]: row for row in protocol["categories"]}
    if tuple(row["category"] for row in protocol["categories"]) != EXPECTED_CATEGORIES:
        raise ValueError("expanded categories differ from the supported contract")
    source_map = {(str(row["factory"]), int(row["seed"])): row for row in source_records}

    if args.selection:
        selection = json.loads(contained(args.selection).read_text(encoding="utf-8"))
        frozen = list(selection["selection"])
    else:
        frozen = []
        for category in protocol["categories"]:
            for seed in protocol["seeds"]:
                record = source_map[(category["factory"], int(seed))]
                frozen.append(
                    {
                        "sample_id": f"infinite_mobility__{category['category']}__seed_{seed:03d}",
                        "category_id": category["category_id"],
                        "category": category["category"],
                        "common_category": category["common_category"],
                        "factory": category["factory"],
                        "seed": int(seed),
                        "selection_rank": int(seed),
                        "source_terminal_status": record["status"],
                    }
                )
        selection = {
            "protocol_id": protocol["protocol_id"],
            "selection_rule": protocol["selection_rule"],
            "failure_policy": protocol["failure_policy"],
            "selection": frozen,
        }
    frozen.sort(key=lambda row: (EXPECTED_CATEGORIES.index(row["category"]), int(row["seed"])))
    if len(frozen) != 150 or Counter(row["category"] for row in frozen) != Counter({category: 30 for category in EXPECTED_CATEGORIES}):
        raise ValueError("expanded selection must contain 30 rows in each of five categories")
    for row in frozen:
        category = category_by_factory.get(row["factory"])
        source = source_map.get((row["factory"], int(row["seed"])))
        if category is None or source is None or category["category"] != row["category"] or source["status"] != row["source_terminal_status"]:
            raise ValueError(f"frozen selection is inconsistent with source identity/status: {row}")

    selection_path = output / "frozen_selection.json"
    write_json(selection_path, selection)
    shutil.copyfile(protocol_path, output / "protocol_snapshot.json")
    shutil.copyfile(ontology_path, output / "ontology_protocol_snapshot.json")
    shutil.copyfile(source_protocol_path, output / "source_protocol_snapshot.json")

    manifest = []
    structure_records = []
    alignment_records = []
    role_assignments = []
    role_audits = []
    for selected in frozen:
        record = source_map[(selected["factory"], int(selected["seed"]))]
        validation = hierarchy.evaluate_case(input_root, record)
        row: dict[str, Any] = {
            "method": "Infinite Mobility",
            **selected,
            "available": validation["evaluation_status"] == "PASS",
            "baseline_status": validation["baseline_status"],
            "evaluation_status": validation["evaluation_status"],
            "case_dir": validation["case_dir"],
            "unavailable_reason": None if validation["evaluation_status"] == "PASS" else validation.get("reason", validation["evaluation_status"]),
        }
        if row["available"]:
            urdf = contained(input_root / validation["urdf_path"])
            case_dir = contained(input_root / validation["case_dir"])
            package_root = contained(case_dir / "package")
            recomputed_package_hash = baseline.package_sha256(package_root)
            if recomputed_package_hash != validation["recorded_package_sha256"]:
                raise ValueError(f"package hash mismatch: {selected['factory']}/{selected['seed']}")
            row.update(
                {
                    "urdf_path": str(urdf),
                    "urdf_sha256": validation["urdf_sha256"],
                    "recorded_package_sha256": validation["recorded_package_sha256"],
                    "recomputed_package_sha256": recomputed_package_hash,
                }
            )
        else:
            row.update({"urdf_path": None, "urdf_sha256": None, "recorded_package_sha256": validation.get("recorded_package_sha256"), "recomputed_package_sha256": None})
        manifest.append(row)

        structure = dict(row)
        structure["evaluated"] = False
        alignment = dict(row)
        alignment["evaluation_complete"] = False
        if row["available"]:
            urdf = Path(row["urdf_path"])
            try:
                structure.update(shared.analyze_urdf(urdf))
                structure["evaluated"] = True
                structure["evaluation_error"] = None
            except Exception as exc:  # noqa: BLE001
                structure["evaluation_error"] = f"{type(exc).__name__}: {exc}"
            try:
                package_root = input_root / row["case_dir"] / "package"
                labels, role_audit, assignments = recover_link_labels(urdf, package_root)
                if not role_audit["recoverable"]:
                    raise ValueError(f"package role mapping not recoverable: {role_audit}")
                alignment.update(evaluate_urdf(urdf, row["category"], ontology, labels))
                alignment["evaluation_complete"] = True
                alignment["evaluation_error"] = None
                alignment["prediction_side_role_label_count"] = len(labels)
                role_audits.append({"sample_id": row["sample_id"], "category": row["category"], "factory": row["factory"], "seed": row["seed"], **role_audit})
                role_assignments.extend({"sample_id": row["sample_id"], "category": row["category"], "factory": row["factory"], "seed": row["seed"], **assignment} for assignment in assignments)
            except Exception as exc:  # noqa: BLE001
                alignment["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        else:
            structure["evaluation_error"] = row["unavailable_reason"]
            alignment["evaluation_error"] = row["unavailable_reason"]
        structure_records.append(structure)
        alignment_records.append(alignment)

    paths = {
        "manifest": output / "evaluation_manifest.jsonl",
        "structure_records": output / "structure_records.jsonl",
        "alignment_records": output / "partnet_alignment_records.jsonl",
        "role_assignments": output / "role_assignments.jsonl",
        "role_audits": output / "role_assignment_audits.jsonl",
    }
    write_jsonl(paths["manifest"], manifest)
    write_jsonl(paths["structure_records"], structure_records)
    write_jsonl(paths["alignment_records"], alignment_records)
    write_jsonl(paths["role_assignments"], role_assignments)
    write_jsonl(paths["role_audits"], role_audits)
    evaluated_structure = [row for row in structure_records if row.get("evaluated")]
    per_category_structure, topology_macro = category_macro_topology(structure_records)
    per_category_alignment = {category: aggregate_alignment([row for row in alignment_records if row["category"] == category]) for category in EXPECTED_CATEGORIES}
    old_nested = load_old_nested(old_panel)
    nested_rows = [row for row in manifest if int(row["seed"]) in NESTED_SEEDS]
    nested_checks = []
    for row in nested_rows:
        old = old_nested.get((row["category"], int(row["seed"])))
        nested_checks.append(
            {
                "sample_id": row["sample_id"],
                "present_in_old_panel": old is not None,
                "urdf_sha256_matches": old is not None and old.get("urdf_sha256") == row.get("urdf_sha256"),
                "availability_matches": old is not None and (old.get("urdf_sha256") is not None) == bool(row["available"]),
            }
        )
    source_status_counts = dict(sorted(Counter(row["source_terminal_status"] for row in frozen).items()))
    summary = {
        "protocol_id": protocol["protocol_id"],
        "display_name": "Infinite Mobility expanded N=150",
        "requested_count": 150,
        "requested_per_category": 30,
        "available_count": sum(bool(row["available"]) for row in manifest),
        "source_terminal_status_counts": source_status_counts,
        "failure_seeds_by_category": {},
        "categories": list(EXPECTED_CATEGORIES),
        "structure": {
            "overall": shared.aggregate(evaluated_structure, requested_count=150),
            "per_category": per_category_structure,
            "category_macro_topology_consistency": topology_macro,
        },
        "partnet_ontology_alignment": {
            "overall": aggregate_alignment(alignment_records),
            "per_category": per_category_alignment,
            "claim_boundary": protocol["claim_boundary"],
            "package_role_labels_are_independent_gold": False,
        },
        "nested_original_n30": {
            "requested_count": len(nested_rows),
            "all_present_in_old_panel": all(row["present_in_old_panel"] for row in nested_checks),
            "all_urdf_hashes_match": all(row["urdf_sha256_matches"] for row in nested_checks),
            "all_availability_matches": all(row["availability_matches"] for row in nested_checks),
        },
        "oven_boundary": protocol["oven_boundary"],
        "blender_invoked": False,
        "paper_ready": True,
    }
    summary["failure_seeds_by_category"] = {category: [int(row["seed"]) for row in manifest if row["category"] == category and not row["available"]] for category in EXPECTED_CATEGORIES}
    summary_path = output / "summary.json"
    write_json(summary_path, summary)
    summary["hashes"] = {
        **{f"{name}_sha256": sha256_file(path) for name, path in paths.items()},
        "frozen_selection_sha256": sha256_file(selection_path),
        "protocol_snapshot_sha256": sha256_file(output / "protocol_snapshot.json"),
        "ontology_protocol_snapshot_sha256": sha256_file(output / "ontology_protocol_snapshot.json"),
        "source_protocol_snapshot_sha256": sha256_file(output / "source_protocol_snapshot.json"),
        "source_runtime_manifest_sha256": sha256_file(input_root / "manifest.json"),
        "source_runtime_records_sha256": sha256_file(input_root / "records.json"),
        "runner_sha256": sha256_file(Path(__file__)),
    }
    write_json(summary_path, summary)

    checks = {
        "selection_has_150_rows": len(frozen) == 150,
        "thirty_per_category": Counter(row["category"] for row in frozen) == Counter({category: 30 for category in EXPECTED_CATEGORIES}),
        "seeds_are_0_to_29_per_category": all(sorted(int(row["seed"]) for row in frozen if row["category"] == category) == list(EXPECTED_SEEDS) for category in EXPECTED_CATEGORIES),
        "selection_precedes_content_evaluation": protocol["selection_rule"].startswith("use every pre-frozen numeric seed"),
        "source_statuses_preserved": all(source_map[(row["factory"], int(row["seed"]))]["status"] == row["source_terminal_status"] for row in frozen),
        "requested_denominator_preserved": len(manifest) == len(structure_records) == len(alignment_records) == 150,
        "available_count_is_146": summary["available_count"] == 146,
        "cabinet_timeout_seeds_are_8_19_20_23": summary["failure_seeds_by_category"]["storage_furniture"] == [8, 19, 20, 23],
        "other_categories_have_no_failures": all(not summary["failure_seeds_by_category"][category] for category in EXPECTED_CATEGORIES if category != "storage_furniture"),
        "all_available_packages_hash_match": all(not row["available"] or row["recorded_package_sha256"] == row["recomputed_package_sha256"] for row in manifest),
        "all_available_role_mappings_recoverable": len(role_audits) == summary["available_count"] and all(row["recoverable"] for row in role_audits),
        "nested_original_has_30_rows": len(nested_rows) == 30,
        "nested_original_all_present": summary["nested_original_n30"]["all_present_in_old_panel"],
        "nested_original_hashes_match": summary["nested_original_n30"]["all_urdf_hashes_match"],
        "nested_original_availability_matches": summary["nested_original_n30"]["all_availability_matches"],
        "oven_not_in_categories": "oven" not in EXPECTED_CATEGORIES and all(row["factory"] != "OvenFactory" for row in frozen),
    }
    verification = {"passed": all(checks.values()), "checks": checks, "summary_sha256": sha256_file(summary_path), "runner_sha256": sha256_file(Path(__file__))}
    write_json(output / "verification.json", verification)
    if not verification["passed"]:
        raise ValueError(f"verification failed: {checks}")
    report = [
        "# Infinite Mobility expanded hierarchy N=150",
        "",
        "The panel freezes seeds 0-29 before content evaluation for five exact Table 3 categories. Failures are retained without replacement.",
        "",
        f"- Available/evaluated structural assets: {summary['available_count']}/150.",
        f"- Valid trees/requested: {summary['structure']['overall']['valid_tree_count']}/150.",
        f"- Failed storage-furniture seeds: {summary['failure_seeds_by_category']['storage_furniture']}.",
        f"- Mean nodes/depth/movable joints on valid assets: {summary['structure']['overall']['node_count_mean']:.3f}/{summary['structure']['overall']['semantic_depth_mean']:.3f}/{summary['structure']['overall']['movable_edge_count_mean']:.3f}.",
        f"- Alignment role coverage/scorable/CW induced Edge F1/semantic-parent requested: {100*summary['partnet_ontology_alignment']['overall']['semantic_role_coverage_requested_macro']:.2f}%/{summary['partnet_ontology_alignment']['overall']['scorable_count']}/150/{100*summary['partnet_ontology_alignment']['overall']['coverage_weighted_induced_edge_f1_requested_macro']:.2f}%/{100*summary['partnet_ontology_alignment']['overall']['semantic_nesting_accuracy_requested_macro']:.2f}%.",
        "- Seeds 0-5 are a strict byte-verified nested copy of the original correctness N=30 panel.",
        "- OvenFactory is excluded; MicrowaveFactory supplies the microwave correctness category.",
        "",
        "Package-local roles are prediction-side labels, not independent gold. The PartNet result is an ontology-alignment proxy rather than kinematic hierarchy correctness.",
    ]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "requested": 150, "available": summary["available_count"], "valid": summary["structure"]["overall"]["valid_tree_count"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
