#!/usr/bin/env python3
"""Extended shared hierarchy metrics for the frozen official LAM Main30."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from statistics import mean
from typing import Any

from hierarchy_extended_metrics import analyze_urdf, aggregate, topology_consistency


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/lam"
DEFAULT_OUTPUT = DEFAULT_INPUT
SHARED_MAPPING = EXP_ROOT / "reference/infinite_mobility_hierarchy_common_categories_v1.json"
SHARED_EVALUATOR = Path(__file__).resolve().parent / "hierarchy_extended_metrics.py"
BASE_MANIFEST_SHA256 = "9e7d7707d255130d70a286a40acdc9e27cca4586398bbdfedce594e45091fd35"
BASE_SELECTION_PROTOCOL_SHA256 = "01f71d9ba2168b9d78d1e7391fc9c9427afaf509b23ec0708d53a2fd412c0a62"
SHARED_MAPPING_SHA256 = "1b791c9d445f5b6074c70ed87d02a918496a5f731237a1a8c83d2d473e472eca"
SHARED_EVALUATOR_SHA256 = "812c481a3d738b8d17893559716c275b6152f22942f37f06df2231d65f36fcd4"
EXPECTED_CLASSES = {
    "storage_furniture_cabinet",
    "table",
    "refrigerator",
    "dishwasher",
    "oven",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def macro_topology(per_category: dict[str, dict[str, Any]]) -> dict[str, Any]:
    rows = list(per_category.values())
    return {
        "category_count": len(rows),
        "sample_count_total": sum(int(row["sample_count"]) for row in rows),
        "unique_signature_rate_macro": mean(float(row["unique_signature_rate"]) for row in rows),
        "mode_rate_macro": mean(float(row["mode_rate"]) for row in rows),
        "pairwise_exact_rate_macro": mean(float(row["pairwise_exact_rate"]) for row in rows),
        "normalized_entropy_macro": mean(float(row["normalized_entropy"]) for row in rows),
        "aggregation": "unweighted mean over five shared categories",
    }


def render_report(summary: dict[str, Any], gold: dict[str, Any]) -> str:
    overall = summary["overall"]
    topology = summary["category_normalized_topology_consistency"]["macro"]
    rows = [
        "# LAM Main30 Extended Hierarchy Metrics",
        "",
        f"- Status: **{summary['status']}**",
        f"- Frozen selection N: `{overall['requested_count']}`",
        f"- Evaluated N: `{overall['evaluated_count']}`",
        f"- Valid-tree N for structural means: `{overall['valid_tree_count']}`",
        f"- Shared evaluator: `{summary['shared_evaluator_sha256']}`",
        "",
        "## Structure",
        "",
        f"- Nodes: {overall['node_count_mean']:.3f}/valid asset; {overall['node_count_total']:.0f} total",
        f"- Edges: {overall['edge_count_mean']:.3f}/valid asset; {overall['edge_count_total']:.0f} total",
        f"- Leaves: {overall['leaf_count_mean']:.3f}/valid asset; leaf ratio {overall['leaf_ratio_mean']:.3f}",
        f"- Branching nodes: {overall['branching_node_count_mean']:.3f}/valid asset; ratio {overall['branching_node_ratio_mean']:.3f}",
        f"- Mean internal out-degree: {overall['mean_internal_out_degree_mean']:.3f}; max out-degree mean {overall['max_out_degree_mean']:.3f}",
        f"- Movable edges: {overall['movable_edge_count_mean']:.3f}/valid asset; {overall['movable_edge_count_total']:.0f} total",
        f"- Fixed edges: {overall['fixed_edge_count_mean']:.3f}/valid asset; {overall['fixed_edge_count_total']:.0f} total",
        f"- Movable edge ratio: {overall['movable_edge_ratio_mean']:.3f}",
        f"- Movable depth: {overall['movable_depth_mean']:.3f}",
        "",
        "## Coverage and defects",
        "",
        f"- Largest-component node coverage: {overall['largest_component_node_rate_mean_evaluated']:.3f} mean over all evaluated assets",
        f"- Valid joint-endpoint coverage: {overall['valid_joint_endpoint_rate_mean_evaluated']:.3f} mean over all evaluated assets",
        f"- Single-root: {overall['single_root_count']}/{overall['evaluated_count']}; root defects {overall['root_defect_count']}",
        f"- Connected: {overall['connected_count']}/{overall['evaluated_count']}; component defects {overall['component_defect_count']}",
        f"- Cycle/malformed/multi-parent defects: {overall['cycle_defect_count']}/{overall['malformed_edge_asset_count']}/{overall['multi_parent_asset_count']}",
        f"- Unnamed-or-duplicate-link defects: {overall['unnamed_or_duplicate_link_asset_count']}",
        "",
        "## Category-normalized topology consistency",
        "",
        "The shared canonical signature removes semantic names and sibling order while retaining rooted shape, exact joint type, and visual/group node labels.",
        "",
        f"- Valid-tree samples: {topology['sample_count_total']}/30",
        f"- Unique-signature rate: {topology['unique_signature_rate_macro']:.3f} category macro",
        f"- Mode rate: {topology['mode_rate_macro']:.3f} category macro",
        f"- Pairwise exact rate: {topology['pairwise_exact_rate_macro']:.3f} category macro",
        f"- Normalized entropy: {topology['normalized_entropy_macro']:.3f} category macro",
        "",
        "## Per-category",
        "",
        "| Category | Valid | Nodes | Edges | Leaves | Branches | Movable | Fixed | Root defects | Component defects | Topology mode | Pair exact |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for category in sorted(summary["per_category"]):
        structure = summary["per_category"][category]["structure"]
        consistency = summary["per_category"][category]["topology_consistency"]
        rows.append(
            f"| {category} | {structure['valid_tree_count']}/6 | {structure['node_count_mean']:.3f} | {structure['edge_count_mean']:.3f} | {structure['leaf_count_mean']:.3f} | {structure['branching_node_count_mean']:.3f} | {structure['movable_edge_count_mean']:.3f} | {structure['fixed_edge_count_mean']:.3f} | {structure['root_defect_count']} | {structure['component_defect_count']} | {consistency['mode_rate']:.3f} | {consistency['pairwise_exact_rate']:.3f} |"
        )
    rows.extend(
        [
            "",
            "## Correctness gold preflight",
            "",
            f"- Parent-Child Edge F1: {gold['metrics']['parent_child_edge_f1']['status']}",
            f"- Hierarchy Exact Match: {gold['metrics']['hierarchy_exact_match']['status']}",
            f"- Semantic Nesting Accuracy: {gold['metrics']['semantic_nesting_accuracy']['status']}",
            "",
            "The official prompts provide no per-output semantic node inventory or edge gold. LAM's links_hierarchy_json is generated by the evaluated method and exists for only 16/30 selected rows; it cannot serve as independent reference gold.",
            "",
            "## Claim boundary",
            "",
            "Structural means follow the shared evaluator and use valid trees only. Defect and coverage counts use every evaluated asset. Topology consistency is within-category release diversity, not cross-seed generator consistency or semantic correctness.",
        ]
    )
    return "\n".join(rows) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workspace-root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    workspace = args.workspace_root.resolve(strict=True)
    input_root = args.input.resolve(strict=True)
    output_root = args.output.resolve(strict=False)
    shared_mapping = SHARED_MAPPING.resolve(strict=True)
    shared_evaluator = SHARED_EVALUATOR.resolve(strict=True)
    for path in (input_root, output_root, shared_mapping, shared_evaluator):
        path.relative_to(workspace)

    manifest_path = (input_root / "manifest.jsonl").resolve(strict=True)
    selection_path = (input_root / "selection_protocol.json").resolve(strict=True)
    if sha256_file(manifest_path) != BASE_MANIFEST_SHA256:
        raise ValueError("frozen LAM Main30 manifest hash mismatch")
    if sha256_file(selection_path) != BASE_SELECTION_PROTOCOL_SHA256:
        raise ValueError("frozen LAM selection protocol hash mismatch")
    if sha256_file(shared_mapping) != SHARED_MAPPING_SHA256:
        raise ValueError("shared hierarchy mapping protocol hash mismatch")
    if sha256_file(shared_evaluator) != SHARED_EVALUATOR_SHA256:
        raise ValueError("shared extended evaluator hash mismatch")

    manifest = [
        json.loads(line)
        for line in manifest_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    expected_counts = Counter({category: 6 for category in EXPECTED_CLASSES})
    if len(manifest) != 30 or Counter(row["shared_class"] for row in manifest) != expected_counts:
        raise ValueError("selection changed: expected five classes x six rows")

    base_records_path = (input_root / "records.jsonl").resolve(strict=True)
    base_records = [
        json.loads(line)
        for line in base_records_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    links_json_present = sum(bool(row["links_hierarchy_json_present"]) for row in base_records)

    records: list[dict[str, Any]] = []
    for item in manifest:
        urdf_path = Path(item["final_urdf"]).resolve(strict=True)
        urdf_path.relative_to(workspace)
        if sha256_file(urdf_path) != item["final_urdf_sha256"]:
            raise ValueError(f"selected URDF hash mismatch: {item['sample_id']}")
        records.append({**item, **analyze_urdf(urdf_path)})

    per_category = {}
    topology_rows = {}
    for category in sorted(EXPECTED_CLASSES):
        category_records = [row for row in records if row["shared_class"] == category]
        consistency = topology_consistency(category_records)
        per_category[category] = {
            "structure": aggregate(category_records, requested_count=6),
            "topology_consistency": consistency,
        }
        topology_rows[category] = consistency

    gold_preflight = {
        "protocol": "nano3d_hierarchy_independent_gold_preflight_v1",
        "status": "UNSUPPORTED_NO_INDEPENDENT_GOLD",
        "selected_asset_count": len(records),
        "shared_mapping_protocol": str(shared_mapping),
        "shared_mapping_protocol_sha256": SHARED_MAPPING_SHA256,
        "available_evidence": {
            "official_category_prompt_lists": "available, but no per-output semantic node inventory or canonical edge set",
            "lam_urdf": "prediction under evaluation, not gold",
            "lam_links_hierarchy_json": f"output-derived; present for {links_json_present}/30 selected rows; prohibited as gold",
            "independent_instance_aligned_hierarchy_annotations": 0,
            "independent_alias_mappings": 0,
            "frozen_alternative_topology_rules": 0,
        },
        "required_gold_contract": {
            "node_roles": "independent semantic node inventory per selected release row",
            "alias_mapping": "prediction link to gold-role mapping independent of LAM output",
            "edge_gold": "allowed parent-child role edges per row",
            "wrapper_policy": "shared fixed/group wrapper collapse rules",
            "alternative_topologies": "enumerated compatible trees for categories with legal variation",
            "adjudication": "independent annotation and conflict resolution",
        },
        "metrics": {
            "parent_child_edge_f1": {
                "status": "N/A",
                "reason": "no independent node mapping and gold parent-child edge set",
            },
            "hierarchy_exact_match": {
                "status": "N/A",
                "reason": "no independent canonical tree or allowed alternative topology set",
            },
            "semantic_nesting_accuracy": {
                "status": "N/A",
                "reason": "no independent semantic parent-role annotations",
            },
        },
        "prohibited_shortcuts": [
            "treating parseable or valid URDF as semantic correctness",
            "using LAM-generated links_hierarchy_json as reference gold",
            "using another generated LAM row as category gold",
            "forcing broad-category assets to one topology without compatibility rules",
        ],
    }
    summary = {
        "protocol": "nano3d_hierarchy_lam_official_extended_v1",
        "status": "VALIDATED",
        "selection_unchanged": True,
        "selection_manifest_sha256": BASE_MANIFEST_SHA256,
        "selection_protocol_sha256": BASE_SELECTION_PROTOCOL_SHA256,
        "shared_mapping_protocol": str(shared_mapping),
        "shared_mapping_protocol_sha256": SHARED_MAPPING_SHA256,
        "shared_evaluator": str(shared_evaluator),
        "shared_evaluator_sha256": SHARED_EVALUATOR_SHA256,
        "runner_sha256": sha256_file(Path(__file__).resolve()),
        "overall": aggregate(records, requested_count=30),
        "per_category": per_category,
        "category_normalized_topology_consistency": {
            "per_category": topology_rows,
            "macro": macro_topology(topology_rows),
            "mapping": "shared name-free rooted topology; exact joint type and visual/group labels retained",
            "claim_boundary": "release topology diversity only; not semantic correctness",
        },
        "correctness_metrics": {
            "parent_child_edge_f1": None,
            "hierarchy_exact_match": None,
            "semantic_nesting_accuracy": None,
            "preflight_status": gold_preflight["status"],
        },
    }

    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "extended_records.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in records),
        encoding="utf-8",
    )
    write_json(output_root / "extended_summary.json", summary)
    write_json(output_root / "hierarchy_gold_preflight.json", gold_preflight)
    (output_root / "extended_report.md").write_text(
        render_report(summary, gold_preflight), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
