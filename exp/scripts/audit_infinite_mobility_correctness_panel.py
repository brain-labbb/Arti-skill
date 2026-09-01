#!/usr/bin/env python3
"""Freeze and audit the Infinite Mobility hierarchy correctness panel."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
from typing import Any
import xml.etree.ElementTree as ET

import hierarchy_extended_metrics as shared
import run_infinite_mobility_baseline as baseline
import run_infinite_mobility_hierarchy as hierarchy


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = (
    EXP_ROOT / "reference/infinite_mobility_hierarchy_correctness_panel_v1.json"
)
DEFAULT_ONTOLOGY = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_SOURCE_PROTOCOL = EXP_ROOT / "reference/infinite_mobility_protocol_v1.json"
DEFAULT_INPUT = EXP_ROOT / "runtime/infinite_mobility_v1"
DEFAULT_PAPER_ROOT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/infinite_mobility"
DEFAULT_PREVIOUS_MANIFEST = DEFAULT_PAPER_ROOT / "cohort_manifest.json"
DEFAULT_PROVENANCE = DEFAULT_PAPER_ROOT / "provenance.json"
DEFAULT_OUTPUT = DEFAULT_PAPER_ROOT / "correctness_panel"
DEFAULT_LOCAL_SOURCE = EXP_ROOT.parent / ".cache/Infinite-Mobility"
DEFAULT_OFFICIAL_SOURCE = EXP_ROOT / "baselines/Infinite-Mobility-official"
MICROWAVE_SOURCE = "infinigen/assets/objects/appliances/microwave.py"
OPAQUE_LINK = re.compile(r"^l_(\d+)$")
BASELINE_RUNNER = Path(baseline.__file__).resolve()
HIERARCHY_VALIDATOR = Path(hierarchy.__file__).resolve()
SHARED_EVALUATOR = Path(shared.__file__).resolve()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_role(part_name: str) -> str:
    role = part_name.strip().lower()
    if role.endswith("_part"):
        role = role[:-5]
    aliases = {"microwave_body": "body", "botton": "button"}
    return aliases.get(role, role)


def flatten_parts(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, list):
        raise ValueError("data_infos root must be a list")
    parts: list[dict[str, Any]] = []
    for group in payload:
        if not isinstance(group, dict) or not isinstance(group.get("part"), list):
            raise ValueError("data_infos group must contain a part list")
        for row in group["part"]:
            if not isinstance(row, dict):
                raise ValueError("data_infos part row must be an object")
            parts.append(row)
    return parts


def audit_role_mapping(
    urdf_path: Path,
    package_root: Path,
    local_source: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata_files = sorted(package_root.glob("data_infos_*.json"))
    if len(metadata_files) != 1:
        raise ValueError(f"expected one data_infos file, found {len(metadata_files)}")
    metadata_path = hierarchy.ensure_contained(metadata_files[0], package_root)
    parts = flatten_parts(json.loads(metadata_path.read_text(encoding="utf-8")))
    filenames = [str(row.get("file_name", "")) for row in parts]
    counts = Counter(filenames)
    duplicate_filenames = sorted(name for name, count in counts.items() if count > 1)
    by_filename = {
        str(row["file_name"]): row
        for row in parts
        if row.get("file_name") and counts[str(row["file_name"])] == 1
    }

    xml_root = ET.parse(urdf_path).getroot()
    assignments: list[dict[str, Any]] = []
    missing: list[dict[str, str]] = []
    index_disagreements: list[dict[str, str]] = []
    referenced_filenames: list[str] = []
    for link in xml_root.findall("link"):
        link_name = link.attrib.get("name", "")
        for mesh in link.findall("visual/geometry/mesh"):
            mesh_path = mesh.attrib.get("filename", "")
            mesh_basename = Path(mesh_path).name
            referenced_filenames.append(mesh_basename)
            metadata = by_filename.get(mesh_basename)
            if metadata is None:
                missing.append(
                    {"link_name": link_name, "mesh_filename": mesh_path}
                )
                continue
            match = OPAQUE_LINK.fullmatch(link_name)
            mesh_stem = Path(mesh_basename).stem
            index_agreement = bool(match and match.group(1) == mesh_stem)
            if not index_agreement:
                index_disagreements.append(
                    {
                        "link_name": link_name,
                        "mesh_filename": mesh_path,
                        "metadata_file_name": str(metadata["file_name"]),
                    }
                )
            raw_role = str(metadata.get("part_name", ""))
            assignments.append(
                {
                    "link_name": link_name,
                    "mesh_filename": mesh_path,
                    "metadata_file_name": str(metadata["file_name"]),
                    "raw_part_name": raw_role,
                    "predicted_role": normalize_role(raw_role),
                    "opaque_link_index_matches_mesh_stem": index_agreement,
                    "assignment_status": "PREDICTION_ONLY_NOT_GOLD",
                }
            )

    referenced_set = set(referenced_filenames)
    unreferenced_rows = [
        {
            "file_name": str(row.get("file_name", "")),
            "part_name": str(row.get("part_name", "")),
        }
        for row in parts
        if str(row.get("file_name", "")) not in referenced_set
    ]
    direct_paths: list[dict[str, Any]] = []
    for row in parts:
        raw_path = str(row.get("file_obj_path", ""))
        candidate = hierarchy.ensure_contained(
            local_source / raw_path, local_source, must_exist=False
        )
        direct_paths.append(
            {
                "file_name": str(row.get("file_name", "")),
                "file_obj_path": raw_path,
                "currently_exists": candidate.is_file(),
            }
        )
    informative = [
        row for row in assignments if row["predicted_role"] not in {"", "unknown", "part"}
    ]
    audit = {
        "metadata_path": metadata_path.relative_to(package_root).as_posix(),
        "metadata_sha256": sha256(metadata_path),
        "metadata_row_count": len(parts),
        "duplicate_metadata_filenames": duplicate_filenames,
        "urdf_visual_mesh_count": len(referenced_filenames),
        "mapped_visual_mesh_count": len(assignments),
        "mapping_coverage": (
            len(assignments) / len(referenced_filenames) if referenced_filenames else None
        ),
        "informative_predicted_role_count": len(informative),
        "informative_predicted_role_coverage": (
            len(informative) / len(referenced_filenames) if referenced_filenames else None
        ),
        "opaque_index_agreement_count": sum(
            bool(row["opaque_link_index_matches_mesh_stem"]) for row in assignments
        ),
        "missing_metadata_matches": missing,
        "index_disagreements": index_disagreements,
        "unreferenced_metadata_rows": unreferenced_rows,
        "direct_file_obj_path_exists_count": sum(
            bool(row["currently_exists"]) for row in direct_paths
        ),
        "direct_file_obj_path_missing_count": sum(
            not bool(row["currently_exists"]) for row in direct_paths
        ),
        "direct_path_note": (
            "file_obj_path records the pre-move staging location. Recovery uses the "
            "package-local metadata file_name joined to the normalized URDF mesh basename."
        ),
        "role_vocabulary": dict(
            sorted(Counter(row["predicted_role"] for row in assignments).items())
        ),
        "recoverable_for_prediction": bool(
            referenced_filenames
            and len(assignments) == len(referenced_filenames)
            and not duplicate_filenames
            and not missing
            and not index_disagreements
        ),
        "gold_eligible": False,
        "gold_rejection_reason": (
            "data_infos is generated package metadata, not an independent annotation"
        ),
    }
    return audit, assignments


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--source-protocol", type=Path, default=DEFAULT_SOURCE_PROTOCOL)
    parser.add_argument("--ontology", type=Path, default=DEFAULT_ONTOLOGY)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--previous-manifest", type=Path, default=DEFAULT_PREVIOUS_MANIFEST)
    parser.add_argument("--provenance", type=Path, default=DEFAULT_PROVENANCE)
    parser.add_argument("--local-source", type=Path, default=DEFAULT_LOCAL_SOURCE)
    parser.add_argument("--official-source", type=Path, default=DEFAULT_OFFICIAL_SOURCE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    workspace_root = EXP_ROOT.parent.resolve(strict=True)
    protocol_path = hierarchy.ensure_contained(args.protocol, workspace_root)
    source_protocol_path = hierarchy.ensure_contained(
        args.source_protocol, workspace_root
    )
    ontology_path = hierarchy.ensure_contained(args.ontology, workspace_root)
    input_root = hierarchy.ensure_contained(args.input_root, workspace_root)
    previous_manifest_path = hierarchy.ensure_contained(
        args.previous_manifest, workspace_root
    )
    provenance_path = hierarchy.ensure_contained(args.provenance, workspace_root)
    local_source = hierarchy.ensure_contained(args.local_source, workspace_root)
    official_source = hierarchy.ensure_contained(args.official_source, workspace_root)
    hierarchy.ensure_contained(BASELINE_RUNNER, workspace_root)
    hierarchy.ensure_contained(HIERARCHY_VALIDATOR, workspace_root)
    hierarchy.ensure_contained(SHARED_EVALUATOR, workspace_root)
    output_root = hierarchy.ensure_contained(
        args.output_root, workspace_root, must_exist=False
    )

    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    ontology = json.loads(ontology_path.read_text(encoding="utf-8"))
    ontology_contract = protocol["independent_ontology"]
    if sha256(ontology_path) != ontology_contract["sha256"]:
        raise ValueError("independent ontology hash mismatch")
    if ontology.get("protocol_id") != ontology_contract["protocol_id"]:
        raise ValueError("independent ontology protocol mismatch")
    if ontology.get("source", {}).get("commit") != ontology_contract["source_commit"]:
        raise ValueError("independent ontology source commit mismatch")
    ontology_categories = set(ontology["categories"])
    requested_ontology_categories = {
        category["ontology_category"] for category in protocol["categories"]
    }
    if not requested_ontology_categories.issubset(ontology_categories):
        raise ValueError("correctness category is absent from independent ontology")
    source_protocol, source_manifest, source_records = hierarchy.load_and_validate_inputs(
        source_protocol_path, input_root
    )
    if source_protocol["protocol_id"] != protocol["source_seed_protocol"]:
        raise ValueError("source seed protocol mismatch")
    previous_manifest = json.loads(previous_manifest_path.read_text(encoding="utf-8"))
    previous_main = {
        (str(row["factory"]), int(row["seed"])): row
        for row in previous_manifest["selection"]
        if row.get("paper_main")
    }
    prior_provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    if prior_provenance["status"] != "PASS":
        raise ValueError("existing official provenance audit did not pass")
    if prior_provenance["official_commit"] != protocol["official_commit"]:
        raise ValueError("official commit differs from correctness protocol")
    if (
        prior_provenance["runtime_python_source_tree_sha256"]
        != source_manifest["baseline_source_tree_sha256"]
    ):
        raise ValueError("runtime source-tree provenance mismatch")
    local_microwave_source = hierarchy.ensure_contained(
        local_source / MICROWAVE_SOURCE, local_source
    )
    official_microwave_source = hierarchy.ensure_contained(
        official_source / MICROWAVE_SOURCE, official_source
    )
    microwave_source_hash = sha256(local_microwave_source)
    if microwave_source_hash != sha256(official_microwave_source):
        raise ValueError("local MicrowaveFactory source differs from official checkout")

    record_map = {
        (str(row["factory"]), int(row["seed"])): row for row in source_records
    }
    selections: list[dict[str, Any]] = []
    package_audits: list[dict[str, Any]] = []
    role_assignments: list[dict[str, Any]] = []
    role_audits: list[dict[str, Any]] = []
    for category in protocol["categories"]:
        factory = category["factory"]
        for seed in protocol["seeds"]:
            key = (factory, int(seed))
            sample_id = (
                f"infinite_mobility__{category['ontology_category']}__seed_{seed:03d}"
            )
            record = record_map[key]
            validation = hierarchy.evaluate_case(input_root, record)
            if validation["evaluation_status"] != "PASS":
                raise ValueError(f"correctness panel case is not PASS: {key}")
            urdf_path = hierarchy.ensure_contained(
                input_root / str(validation["urdf_path"]), input_root
            )
            case_dir = hierarchy.ensure_contained(
                input_root / str(validation["case_dir"]), input_root
            )
            package_root = hierarchy.ensure_contained(case_dir / "package", input_root)
            recomputed_package_hash = baseline.package_sha256(package_root)
            if recomputed_package_hash != validation["recorded_package_sha256"]:
                raise ValueError(f"package hash mismatch: {key}")
            graph = shared.analyze_urdf(urdf_path)
            if not graph["valid_tree"]:
                raise ValueError(f"shared hierarchy evaluator rejects {key}")
            role_audit, assignments = audit_role_mapping(
                urdf_path, package_root, local_source
            )
            if not role_audit["recoverable_for_prediction"]:
                raise ValueError(f"package-local role mapping is incomplete: {key}")

            previous = previous_main.get(key)
            preserved_previous_selection = factory != "MicrowaveFactory"
            if preserved_previous_selection:
                if previous is None:
                    raise ValueError(f"previous paper-main selection missing {key}")
                if previous["urdf_sha256"] != validation["urdf_sha256"]:
                    raise ValueError(f"previous paper-main URDF hash changed: {key}")
            selection = {
                "sample_id": sample_id,
                "category_id": category["category_id"],
                "common_category": category["common_category"],
                "ontology_category": category["ontology_category"],
                "factory": factory,
                "seed": seed,
                "selection_origin": category["selection_origin"],
                "preserved_previous_selection": preserved_previous_selection,
                "baseline_status": validation["baseline_status"],
                "case_dir": validation["case_dir"],
                "urdf_path": validation["urdf_path"],
                "final_urdf_path": str(urdf_path),
                "urdf_sha256": validation["urdf_sha256"],
                "recorded_package_sha256": validation["recorded_package_sha256"],
                "recomputed_package_sha256": recomputed_package_hash,
                "metadata_path": role_audit["metadata_path"],
                "metadata_sha256": role_audit["metadata_sha256"],
                "valid_tree": graph["valid_tree"],
                "node_count": graph["node_count"],
                "edge_count": graph["edge_count"],
            }
            selections.append(selection)
            package_audits.append(
                {
                    **selection,
                    "process_and_mesh_validation": {
                        "completion_marker_present": validation[
                            "completion_marker_present"
                        ],
                        "mesh_reference_count": validation["mesh_reference_count"],
                        "package_hash_matches_record": True,
                    },
                }
            )
            role_audits.append(
                {
                    "category_id": category["category_id"],
                    "sample_id": sample_id,
                    "factory": factory,
                    "seed": seed,
                    **role_audit,
                }
            )
            role_assignments.extend(
                {
                    "category_id": category["category_id"],
                    "sample_id": sample_id,
                    "factory": factory,
                    "seed": seed,
                    **assignment,
                }
                for assignment in assignments
            )

    if len(selections) != 30:
        raise ValueError(f"expected 30 correctness selections, found {len(selections)}")
    microwave_selections = [
        row for row in selections if row["factory"] == "MicrowaveFactory"
    ]
    if len(microwave_selections) != 6:
        raise ValueError("expected six MicrowaveFactory selections")

    total_visuals = sum(int(row["urdf_visual_mesh_count"]) for row in role_audits)
    total_mapped = sum(int(row["mapped_visual_mesh_count"]) for row in role_audits)
    total_informative = sum(
        int(row["informative_predicted_role_count"]) for row in role_audits
    )
    total_index_agreement = sum(
        int(row["opaque_index_agreement_count"]) for row in role_audits
    )
    total_metadata = sum(int(row["metadata_row_count"]) for row in role_audits)
    total_unreferenced = sum(
        len(row["unreferenced_metadata_rows"]) for row in role_audits
    )
    stale_paths = sum(
        int(row["direct_file_obj_path_missing_count"]) for row in role_audits
    )
    role_vocab = Counter(
        assignment["predicted_role"] for assignment in role_assignments
    )
    microwave_role_audits = [
        row for row in role_audits if row["factory"] == "MicrowaveFactory"
    ]
    per_category = []
    for category in protocol["categories"]:
        category_audits = [
            row for row in role_audits if row["factory"] == category["factory"]
        ]
        visual_count = sum(int(row["urdf_visual_mesh_count"]) for row in category_audits)
        mapped_count = sum(int(row["mapped_visual_mesh_count"]) for row in category_audits)
        informative_count = sum(
            int(row["informative_predicted_role_count"]) for row in category_audits
        )
        per_category.append(
            {
                "category_id": category["category_id"],
                "factory": category["factory"],
                "asset_count": len(category_audits),
                "visual_mesh_count": visual_count,
                "mapped_count": mapped_count,
                "mapping_coverage": mapped_count / visual_count,
                "informative_role_count": informative_count,
                "informative_role_coverage": informative_count / visual_count,
                "recoverable_asset_count": sum(
                    bool(row["recoverable_for_prediction"]) for row in category_audits
                ),
            }
        )

    provenance = {
        "status": "PASS",
        "official_repository": protocol["official_repository"],
        "official_commit": protocol["official_commit"],
        "runtime_source_tree_sha256": source_manifest[
            "baseline_source_tree_sha256"
        ],
        "prior_provenance_sha256": sha256(provenance_path),
        "microwave_source_path": MICROWAVE_SOURCE,
        "microwave_source_sha256": microwave_source_hash,
        "microwave_source_matches_official": True,
        "microwave_asset_count": len(microwave_selections),
        "microwave_package_hash_match_count": sum(
            row["recorded_package_sha256"] == row["recomputed_package_sha256"]
            for row in microwave_selections
        ),
        "blender_invoked": False,
    }
    summary = {
        "protocol_id": protocol["protocol_id"],
        "generated_at": utc_now(),
        "requested_asset_count": 30,
        "validated_asset_count": len(selections),
        "valid_tree_count": sum(bool(row["valid_tree"]) for row in selections),
        "previous_selection_preserved_count": sum(
            bool(row["preserved_previous_selection"]) for row in selections
        ),
        "new_microwave_selection_count": len(microwave_selections),
        "microwave_audit": {
            "asset_count": len(microwave_selections),
            "package_hash_match_count": sum(
                row["recorded_package_sha256"] == row["recomputed_package_sha256"]
                for row in microwave_selections
            ),
            "valid_tree_count": sum(
                bool(row["valid_tree"]) for row in microwave_selections
            ),
            "node_counts": [int(row["node_count"]) for row in microwave_selections],
            "edge_counts": [int(row["edge_count"]) for row in microwave_selections],
            "metadata_row_count": sum(
                int(row["metadata_row_count"]) for row in microwave_role_audits
            ),
            "visual_mesh_count": sum(
                int(row["urdf_visual_mesh_count"]) for row in microwave_role_audits
            ),
            "mapped_visual_mesh_count": sum(
                int(row["mapped_visual_mesh_count"]) for row in microwave_role_audits
            ),
            "opaque_index_agreement_count": sum(
                int(row["opaque_index_agreement_count"])
                for row in microwave_role_audits
            ),
            "unreferenced_metadata_row_count": sum(
                len(row["unreferenced_metadata_rows"])
                for row in microwave_role_audits
            ),
            "stale_direct_file_obj_path_count": sum(
                int(row["direct_file_obj_path_missing_count"])
                for row in microwave_role_audits
            ),
        },
        "package_hash_match_count": sum(
            row["recorded_package_sha256"] == row["recomputed_package_sha256"]
            for row in selections
        ),
        "role_assignment_audit": {
            "status": "RECOVERABLE_FOR_PREDICTION_ONLY",
            "asset_count": len(role_audits),
            "recoverable_asset_count": sum(
                bool(row["recoverable_for_prediction"]) for row in role_audits
            ),
            "metadata_row_count": total_metadata,
            "urdf_visual_mesh_count": total_visuals,
            "mapped_visual_mesh_count": total_mapped,
            "mapping_coverage": total_mapped / total_visuals,
            "opaque_index_agreement_count": total_index_agreement,
            "opaque_index_agreement_coverage": total_index_agreement / total_visuals,
            "informative_predicted_role_count": total_informative,
            "informative_predicted_role_coverage": total_informative / total_visuals,
            "unreferenced_metadata_row_count": total_unreferenced,
            "stale_direct_file_obj_path_count": stale_paths,
            "role_vocabulary": dict(sorted(role_vocab.items())),
            "per_category": per_category,
            "gold_eligible": False,
        },
        "correctness_metrics": {
            "status": "WAIT_FOR_SHARED_SCORER",
            "parent_child_edge_f1": None,
            "hierarchy_exact_match": None,
            "semantic_nesting_accuracy": None,
            "prediction_role_assignment_available": True,
            "independent_ontology_available": True,
            "independent_ontology_sha256": ontology_contract["sha256"],
            "asset_aligned_gold_available": False,
        },
        "provenance": provenance,
    }
    cohort_manifest = {
        "protocol": protocol,
        "protocol_sha256": sha256(protocol_path),
        "independent_ontology_sha256": sha256(ontology_path),
        "independent_ontology_source_commit": ontology_contract["source_commit"],
        "source_protocol_sha256": sha256(source_protocol_path),
        "source_runtime_manifest_sha256": sha256(input_root / "manifest.json"),
        "source_runtime_records_sha256": sha256(input_root / "records.json"),
        "previous_paper_manifest_sha256": sha256(previous_manifest_path),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "baseline_package_hash_runner_sha256": sha256(BASELINE_RUNNER),
        "hierarchy_package_validator_sha256": sha256(HIERARCHY_VALIDATOR),
        "shared_hierarchy_evaluator_sha256": sha256(SHARED_EVALUATOR),
        "selection_count": len(selections),
        "provenance": provenance,
        "selection": selections,
    }
    report = [
        "# Infinite Mobility hierarchy correctness panel audit",
        "",
        "- Cohort: 5 categories x seeds 0-5 = 30 existing assets",
        "- Categories: storage/cabinet, table, refrigerator, dishwasher, microwave",
        f"- Validated packages/URDF trees: {len(selections)}/30",
        "- Previous paper-main selections preserved: 24/24",
        "- MicrowaveFactory existing packages added: 6/6",
        "- Recomputed package hashes matching frozen records: 30/30",
        "- Blender invoked: no",
        (
            "- Microwave node counts: "
            f"{summary['microwave_audit']['node_counts']}; edge counts: "
            f"{summary['microwave_audit']['edge_counts']}"
        ),
        "",
        "## Package-local semantic mapping",
        "",
        f"- Recoverable assets: {summary['role_assignment_audit']['recoverable_asset_count']}/30",
        f"- URDF visual links mapped: {total_mapped}/{total_visuals}",
        f"- Opaque `l_<index>` to mesh-stem agreement: {total_index_agreement}/{total_visuals}",
        f"- Informative predicted roles: {total_informative}/{total_visuals}",
        f"- Metadata rows not referenced by URDF: {total_unreferenced}",
        f"- Stale pre-move `file_obj_path` rows: {stale_paths}/{total_metadata}",
        (
            "- Microwave mapping: "
            f"{summary['microwave_audit']['mapped_visual_mesh_count']}/"
            f"{summary['microwave_audit']['visual_mesh_count']} visual links; "
            f"{summary['microwave_audit']['unreferenced_metadata_row_count']} extra metadata rows"
        ),
        "",
        "The mapping is recovered from package-local `data_infos` file names plus URDF "
        "mesh basenames. It is prediction-side role assignment only and is not independent gold.",
        "",
        "## Correctness status",
        "",
        "- Parent-Child Edge F1: N/A",
        "- Hierarchy Exact Match: N/A",
        "- Semantic Nesting Accuracy: N/A",
        "- Independent ontology: frozen and hash-verified",
        "- Status: `WAIT_FOR_SHARED_SCORER`",
    ]

    output_root.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Any] = {
        "summary.json": summary,
        "cohort_manifest.json": cohort_manifest,
        "package_audit.json": package_audits,
        "role_assignment_audit.json": role_audits,
        "role_assignments.json": role_assignments,
        "provenance.json": provenance,
    }
    for name, payload in outputs.items():
        (output_root / name).write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    (output_root / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    evaluation_rows = [
        {
            "method": "Infinite Mobility",
            "sample_id": row["sample_id"],
            "category": row["ontology_category"],
            "urdf_path": row["final_urdf_path"],
            "urdf_sha256": row["urdf_sha256"],
            "available": True,
            "selection_rank": int(row["seed"]),
        }
        for row in selections
    ]
    (output_root / "evaluation_manifest.jsonl").write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in evaluation_rows),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"outputs={output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
