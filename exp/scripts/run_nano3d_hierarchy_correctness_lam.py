#!/usr/bin/env python3
"""Build and audit the PartNet-aligned LAM hierarchy correctness cohort."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re
import shutil
from typing import Any
import xml.etree.ElementTree as ET

import pyarrow.parquet as pq

from hierarchy_extended_metrics import analyze_urdf
from run_nano3d_hierarchy_paper_lam import rank_key


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASE = EXP_ROOT / "runtime/nano3d_hierarchy_paper/lam"
DEFAULT_DATASET = EXP_ROOT / "baselines/LAM-official-dataset/articulated_code.parquet"
DEFAULT_REPO = EXP_ROOT / "baselines/LAM-official"
DEFAULT_PARTNET = EXP_ROOT.parents[1] / "PartNet_Mobility/data/dataset"
DEFAULT_REFERENCE_ONTOLOGY = EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/lam"
BASE_MANIFEST_SHA256 = "9e7d7707d255130d70a286a40acdc9e27cca4586398bbdfedce594e45091fd35"
DATASET_SHA256 = "ab45cf9154c5d98deef7b9032f622286efecd21228d9d0e9165a2f0811da6764"
DATASET_REVISION = "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0"
REFERENCE_ONTOLOGY_SHA256 = "468eb3e8676454907468aae0e0427d2eb806a38f937682a2b414dee439f8adc4"
MICROWAVE_ALLOWLIST = (
    "advanced_microwave_oven_with_inverter",
    "built_in_microwave_with_convection",
    "commercial_microwave_featuring_programmable_cooking",
    "countertop_microwave_with_turntable",
    "functional_microwave_featuring_articulated",
    "meticulously_crafted_microwave_incorporating",
    "microwave",
    "over_range_microwave_with_exhaust_fan",
    "over_range_microwave_with_exhaust_fan_and_interior",
    "professional_grade_microwave_engineered_with",
    "sensor_controlled_microwave_with_automatic",
    "versatile_microwave_with_hinged",
    "well_designed_microwave_with_moving",
)
PARTNET_CATEGORY_MAP = {
    "storage_furniture_cabinet": "StorageFurniture",
    "table": "Table",
    "refrigerator": "Refrigerator",
    "dishwasher": "Dishwasher",
    "microwave": "Microwave",
}
EVALUATION_CATEGORY_MAP = {
    "storage_furniture_cabinet": "storage_furniture",
    "table": "table",
    "refrigerator": "refrigerator",
    "dishwasher": "dishwasher",
    "microwave": "microwave",
}
MOTION_ROLE_PREFIXES = {"rotation", "translation"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def normalize_tokens(value: str) -> list[str]:
    value = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value)
    return re.findall(r"[a-z0-9]+", value.casefold())


def role_candidates(link_name: str, roles: list[str]) -> tuple[str, list[str]]:
    link_tokens = normalize_tokens(link_name)
    link_joined = "_".join(link_tokens)
    exact = [role for role in roles if "_".join(normalize_tokens(role)) == link_joined]
    if exact:
        return "exact", exact
    core = []
    for role in roles:
        role_tokens = normalize_tokens(role)
        role_core = [token for token in role_tokens if token not in MOTION_ROLE_PREFIXES]
        if role_core and all(token in link_tokens for token in role_core):
            core.append(role)
    if core:
        return "core", core
    heads = [role for role in roles if normalize_tokens(role)[-1] in link_tokens]
    if heads:
        return "head", heads
    return "none", []


def load_partnet_roles(partnet_root: Path) -> tuple[dict[str, Any], dict[str, list[str]]]:
    target_categories = set(PARTNET_CATEGORY_MAP.values())
    object_dirs: dict[str, list[Path]] = defaultdict(list)
    for meta_path in sorted(partnet_root.glob("*/meta.json")):
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        category = meta.get("model_cat")
        if category in target_categories:
            object_dirs[str(category)].append(meta_path.parent)

    audit: dict[str, Any] = {}
    vocabularies: dict[str, list[str]] = {}
    for shared_class, partnet_category in PARTNET_CATEGORY_MAP.items():
        role_counts: Counter[str] = Counter()
        joint_kind_counts: Counter[str] = Counter()
        source_entries: list[str] = []
        missing_semantics = 0
        for object_dir in object_dirs[partnet_category]:
            meta_path = object_dir / "meta.json"
            semantics_path = object_dir / "semantics.txt"
            source_entries.append(
                f"{meta_path.relative_to(partnet_root).as_posix()}:{sha256_file(meta_path)}"
            )
            if not semantics_path.is_file():
                missing_semantics += 1
                continue
            source_entries.append(
                f"{semantics_path.relative_to(partnet_root).as_posix()}:{sha256_file(semantics_path)}"
            )
            for line in semantics_path.read_text(encoding="utf-8").splitlines():
                parts = line.split(maxsplit=2)
                if len(parts) != 3:
                    continue
                joint_kind_counts[parts[1]] += 1
                role_counts[parts[2]] += 1
        roles = sorted(role_counts)
        vocabularies[shared_class] = roles
        audit[shared_class] = {
            "partnet_category": partnet_category,
            "instance_count": len(object_dirs[partnet_category]),
            "missing_semantics_count": missing_semantics,
            "role_counts": dict(sorted(role_counts.items())),
            "joint_kind_counts": dict(sorted(joint_kind_counts.items())),
            "source_set_sha256": hashlib.sha256("\n".join(source_entries).encode("utf-8")).hexdigest(),
        }
    return audit, vocabularies


def render_report(summary: dict[str, Any]) -> str:
    microwave = summary["microwave"]
    mapping = summary["partnet_role_mapability"]
    rows = [
        "# LAM PartNet-Aligned Hierarchy Correctness Cohort",
        "",
        f"- Status: **{summary['status']}**",
        f"- Cohort: five classes x six assets = `{summary['cohort']['row_count']}`",
        f"- Manifest SHA-256: `{summary['cohort']['manifest_sha256']}`",
        "",
        "## Microwave selection",
        "",
        f"- Explicit candidates: {microwave['candidate_count']} rows / {microwave['raw_category_count']} raw categories",
        f"- Candidate tiers: `{json.dumps(microwave['candidate_tier_counts'], sort_keys=True)}`",
        f"- Candidate statuses: `{json.dumps(microwave['candidate_status_counts'], sort_keys=True)}`",
        f"- Selected tiers: `{json.dumps(microwave['selected_tier_counts'], sort_keys=True)}`",
        f"- Selected statuses: `{json.dumps(microwave['selected_status_counts'], sort_keys=True)}`",
        "",
        "Selection used only shared class, raw category, object_release_id, and immutable dataset row index. Status/tier did not participate and failures would not be replaced.",
        "",
        "## Selected microwave rows",
        "",
        "| Rank | Sample | Raw category | Tier | Status | Links | Joints | Movable |",
        "|---:|---|---|---|---|---:|---:|---:|",
    ]
    for row in microwave["selected"]:
        rows.append(
            f"| {row['selection_rank_within_class']} | `{row['sample_id']}` | `{row['raw_category']}` | {row['tier']} | {row['status']} | {row['manifest_n_links']} | {row['manifest_n_joints']} | {row['manifest_n_movable']} |"
        )
    rows.extend(
        [
            "",
            "## URDF and PartNet role audit",
            "",
            f"- URDF present/hash/XML/analyzer: {summary['urdf_audit']['all_gate_pass_count']}/{summary['cohort']['row_count']}",
            f"- Named links: {mapping['link_count']} total",
            f"- Strict exact role names: {mapping['strict_exact_link_count']}/{mapping['link_count']} = {mapping['strict_exact_rate']:.3f}",
            f"- Unique lexical mappings: {mapping['unique_mapping_count']}/{mapping['link_count']} = {mapping['unique_mapping_rate']:.3f}",
            f"- Ambiguous mappings: {mapping['ambiguous_mapping_count']}; unmapped: {mapping['unmapped_count']}",
            "",
            "Roles come only from local PartNet-Mobility `meta.json` and `semantics.txt`. LAM output hierarchy JSON is not read for gold or mapping.",
            "",
            "## Correctness boundary",
            "",
            f"This runner stops at cohort and vocabulary-mapability audit. The independent shared ontology is frozen at `{summary['correctness_status']['shared_reference_ontology_sha256']}`; Parent-Child Edge F1, Exact Match, and Semantic Nesting remain reserved for the shared scorer.",
        ]
    )
    return "\n".join(rows) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--partnet", type=Path, default=DEFAULT_PARTNET)
    parser.add_argument("--reference-ontology", type=Path, default=DEFAULT_REFERENCE_ONTOLOGY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workspace-root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    workspace = args.workspace_root.resolve(strict=True)
    base = args.base.resolve(strict=True)
    dataset = args.dataset.resolve(strict=True)
    repo = args.repo.resolve(strict=True)
    partnet = args.partnet.resolve(strict=True)
    reference_ontology = args.reference_ontology.resolve(strict=True)
    output = args.output.resolve(strict=False)
    for path in (base, dataset, repo, partnet, reference_ontology, output):
        path.relative_to(workspace)
    if sha256_file(base / "manifest.jsonl") != BASE_MANIFEST_SHA256:
        raise ValueError("base LAM Main30 manifest changed")
    if sha256_file(dataset) != DATASET_SHA256:
        raise ValueError("official LAM dataset parquet hash mismatch")
    if sha256_file(reference_ontology) != REFERENCE_ONTOLOGY_SHA256:
        raise ValueError("shared PartNet correctness ontology hash mismatch")

    base_manifest = [
        json.loads(line)
        for line in (base / "manifest.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    retained = [
        row for row in base_manifest
        if row["shared_class"] in PARTNET_CATEGORY_MAP and row["shared_class"] != "microwave"
    ]
    if len(retained) != 24 or Counter(row["shared_class"] for row in retained) != Counter(
        {category: 6 for category in PARTNET_CATEGORY_MAP if category != "microwave"}
    ):
        raise ValueError("base identity selection for first four classes is not intact")

    identity_rows = pq.read_table(dataset, columns=["object_release_id", "category"]).to_pylist()
    microwave_identity = []
    allowlist = set(MICROWAVE_ALLOWLIST)
    for row_index, row in enumerate(identity_rows):
        if row["category"] in allowlist:
            microwave_identity.append({**row, "dataset_row_index": row_index})
    if set(row["category"] for row in microwave_identity) != allowlist:
        raise ValueError("microwave allowlist no longer matches official release")
    ranked = sorted(
        ((rank_key("microwave", row), row) for row in microwave_identity),
        key=lambda item: (item[0], str(item[1]["object_release_id"])),
    )
    selected_identity = ranked[:6]

    full_rows = pq.read_table(dataset).to_pylist()
    candidate_rows = [full_rows[row["dataset_row_index"]] for row in microwave_identity]
    output.mkdir(parents=True, exist_ok=True)
    urdf_dir = output / "selected_urdfs"
    urdf_dir.mkdir(parents=True, exist_ok=True)

    manifest: list[dict[str, Any]] = []
    for item in retained:
        source_urdf = Path(item["final_urdf"]).resolve(strict=True)
        source_urdf.relative_to(workspace)
        destination = urdf_dir / f"{item['sample_id'].replace(':', '-')}.urdf"
        shutil.copyfile(source_urdf, destination)
        if sha256_file(destination) != item["final_urdf_sha256"]:
            raise ValueError(f"retained URDF hash mismatch: {item['sample_id']}")
        manifest.append(
            {
                **item,
                "source_panel": "lam_hierarchy_main30_retained_identity",
                "final_urdf": str(destination),
            }
        )

    selected_microwave = []
    for selection_rank, (selection_hash, identity) in enumerate(selected_identity, 1):
        row = full_rows[identity["dataset_row_index"]]
        object_id = str(row["object_release_id"])
        sample_id = f"row_{identity['dataset_row_index']:04d}:{object_id}"
        urdf_text = row.get("urdf")
        if not isinstance(urdf_text, str) or not urdf_text.strip():
            raise ValueError(f"selected microwave URDF is missing: {sample_id}")
        destination = urdf_dir / f"{sample_id.replace(':', '-')}.urdf"
        destination.write_text(urdf_text, encoding="utf-8")
        manifest_row = {
            "method": "LAM",
            "dataset_revision": DATASET_REVISION,
            "dataset_row_index": identity["dataset_row_index"],
            "sample_id": sample_id,
            "object_release_id": object_id,
            "shared_class": "microwave",
            "raw_category": row["category"],
            "selection_hash": selection_hash,
            "selection_rank_within_class": selection_rank,
            "tier": row.get("tier"),
            "status": row.get("status"),
            "caption": row.get("caption"),
            "official_rel_path": row.get("rel_path"),
            "model": row.get("model"),
            "pipeline": row.get("pipeline"),
            "manifest_n_links": row.get("n_links"),
            "manifest_n_joints": row.get("n_joints"),
            "manifest_n_movable": row.get("n_movable"),
            "source_panel": "identity_only_microwave_selection_v1",
            "final_urdf": str(destination),
            "final_urdf_sha256": sha256_file(destination),
        }
        selected_microwave.append(manifest_row)
        manifest.append(manifest_row)

    manifest.sort(key=lambda row: (row["shared_class"], int(row["selection_rank_within_class"])))
    if len(manifest) != 30 or Counter(row["shared_class"] for row in manifest) != Counter(
        {category: 6 for category in PARTNET_CATEGORY_MAP}
    ):
        raise ValueError("correctness cohort is not five classes x six")
    manifest_path = output / "manifest.jsonl"
    manifest_path.write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in manifest),
        encoding="utf-8",
    )
    cohort_assets = [
        {
            "shared_class": row["shared_class"],
            "sample_id": row["sample_id"],
            "object_release_id": row["object_release_id"],
            "final_urdf": row["final_urdf"],
            "final_urdf_sha256": row["final_urdf_sha256"],
        }
        for row in manifest
    ]
    (output / "cohort_assets.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in cohort_assets),
        encoding="utf-8",
    )
    evaluation_manifest = [
        {
            "method": "LAM",
            "sample_id": row["sample_id"],
            "category": EVALUATION_CATEGORY_MAP[row["shared_class"]],
            "urdf_path": row["final_urdf"],
            "urdf_sha256": row["final_urdf_sha256"],
            "available": (
                Path(row["final_urdf"]).is_file()
                and sha256_file(Path(row["final_urdf"])) == row["final_urdf_sha256"]
            ),
            "selection_rank": int(row["selection_rank_within_class"]),
        }
        for row in manifest
    ]
    (output / "evaluation_manifest.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in evaluation_manifest),
        encoding="utf-8",
    )

    partnet_audit, role_vocabularies = load_partnet_roles(partnet)
    urdf_records = []
    mapping_records = []
    for item in manifest:
        urdf_path = Path(item["final_urdf"]).resolve(strict=True)
        xml_root = ET.parse(urdf_path).getroot()
        link_names = [node.attrib.get("name", "") for node in xml_root.findall("link")]
        analysis = analyze_urdf(urdf_path)
        urdf_records.append(
            {
                "sample_id": item["sample_id"],
                "shared_class": item["shared_class"],
                "tier": item["tier"],
                "status": item["status"],
                "urdf_sha256": sha256_file(urdf_path),
                "urdf_hash_matches_manifest": sha256_file(urdf_path) == item["final_urdf_sha256"],
                "xml_root": xml_root.tag,
                "named_link_count": sum(bool(name) for name in link_names),
                "link_names": link_names,
                "shared_hierarchy_analysis": analysis,
            }
        )
        roles = role_vocabularies[item["shared_class"]]
        for link_name in link_names:
            level, candidates = role_candidates(link_name, roles)
            mapping_records.append(
                {
                    "sample_id": item["sample_id"],
                    "shared_class": item["shared_class"],
                    "partnet_category": PARTNET_CATEGORY_MAP[item["shared_class"]],
                    "link_name": link_name,
                    "match_level": level,
                    "candidate_roles": candidates,
                    "mapping_status": (
                        "unique" if len(candidates) == 1 else "ambiguous" if candidates else "unmapped"
                    ),
                    "is_strict_exact": level == "exact" and len(candidates) == 1,
                }
            )

    microwave_candidate_audit = {
        "protocol": "lam_official_microwave_identity_selection_v1",
        "official_prompt_source": str((repo / "data/val_data/microwave.txt").resolve()),
        "official_prompt_source_sha256": sha256_file(repo / "data/val_data/microwave.txt"),
        "explicit_raw_category_allowlist": list(MICROWAVE_ALLOWLIST),
        "candidate_count": len(candidate_rows),
        "raw_category_count": len(set(row["category"] for row in candidate_rows)),
        "raw_category_counts": dict(sorted(Counter(row["category"] for row in candidate_rows).items())),
        "candidate_tier_counts": dict(sorted(Counter(row.get("tier") for row in candidate_rows).items())),
        "candidate_status_counts": dict(sorted(Counter(row.get("status") for row in candidate_rows).items())),
        "urdf_present_count": sum(isinstance(row.get("urdf"), str) and bool(row["urdf"].strip()) for row in candidate_rows),
        "links_hierarchy_json_present_count": sum(
            isinstance(row.get("links_hierarchy_json"), str) and bool(row["links_hierarchy_json"].strip())
            for row in candidate_rows
        ),
        "selection_rule": "same LAM Main30 identity-only SHA rank; first six; no tier/status filtering; failures retained",
        "selected": selected_microwave,
    }
    link_count = len(mapping_records)
    unique_count = sum(row["mapping_status"] == "unique" for row in mapping_records)
    exact_count = sum(bool(row["is_strict_exact"]) for row in mapping_records)
    summary = {
        "protocol": "nano3d_hierarchy_correctness_lam_partnet_v1",
        "status": "VALIDATED_COHORT_AND_MAPABILITY_AUDIT",
        "cohort": {
            "row_count": len(manifest),
            "class_counts": dict(sorted(Counter(row["shared_class"] for row in manifest).items())),
            "tier_counts": dict(sorted(Counter(row.get("tier") for row in manifest).items())),
            "status_counts": dict(sorted(Counter(row.get("status") for row in manifest).items())),
            "manifest_sha256": sha256_file(manifest_path),
            "retained_base_identity_count": len(retained),
            "new_microwave_identity_count": len(selected_microwave),
        },
        "microwave": {
            **{key: value for key, value in microwave_candidate_audit.items() if key != "selected"},
            "selected_tier_counts": dict(sorted(Counter(row["tier"] for row in selected_microwave).items())),
            "selected_status_counts": dict(sorted(Counter(row["status"] for row in selected_microwave).items())),
            "selected": selected_microwave,
        },
        "urdf_audit": {
            "record_count": len(urdf_records),
            "all_gate_pass_count": sum(
                row["urdf_hash_matches_manifest"] and row["xml_root"] == "robot"
                for row in urdf_records
            ),
        },
        "partnet_ontology": partnet_audit,
        "partnet_role_mapability": {
            "protocol": "strict normalized exact, then motion-prefix-stripped core tokens, then role-head token",
            "link_count": link_count,
            "strict_exact_link_count": exact_count,
            "strict_exact_rate": exact_count / link_count,
            "unique_mapping_count": unique_count,
            "unique_mapping_rate": unique_count / link_count,
            "ambiguous_mapping_count": sum(row["mapping_status"] == "ambiguous" for row in mapping_records),
            "unmapped_count": sum(row["mapping_status"] == "unmapped" for row in mapping_records),
            "uses_lam_links_hierarchy_json": False,
            "claim_boundary": "lexical vocabulary mapability only; not role correctness or hierarchy correctness",
        },
        "correctness_status": {
            "shared_reference_ontology": str(reference_ontology),
            "shared_reference_ontology_sha256": REFERENCE_ONTOLOGY_SHA256,
            "scorer": "NOT RUN: reserved for shared correctness scorer",
            "parent_child_edge_f1": "NOT RUN: shared scorer pending",
            "hierarchy_exact_match": "NOT RUN: shared scorer pending",
            "semantic_nesting_accuracy": "NOT RUN: shared scorer pending",
        },
    }

    write_json(output / "selection_protocol.json", microwave_candidate_audit)
    write_json(output / "partnet_ontology_audit.json", partnet_audit)
    (output / "urdf_audit.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in urdf_records),
        encoding="utf-8",
    )
    (output / "partnet_role_mapability.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in mapping_records),
        encoding="utf-8",
    )
    write_json(output / "summary.json", summary)
    (output / "report.md").write_text(render_report(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
