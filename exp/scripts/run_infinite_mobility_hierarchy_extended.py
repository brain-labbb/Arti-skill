#!/usr/bin/env python3
"""Run shared extended hierarchy metrics on the frozen Infinite Mobility cohort."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from statistics import mean
from typing import Any

import hierarchy_extended_metrics as shared
import run_infinite_mobility_hierarchy as hierarchy


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = (
    EXP_ROOT / "reference/infinite_mobility_hierarchy_common_categories_v1.json"
)
DEFAULT_SOURCE_PROTOCOL = EXP_ROOT / "reference/infinite_mobility_protocol_v1.json"
DEFAULT_INPUT = EXP_ROOT / "runtime/infinite_mobility_v1"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/infinite_mobility"
DEFAULT_COHORT_MANIFEST = DEFAULT_OUTPUT / "cohort_manifest.json"
SHARED_EVALUATOR = Path(shared.__file__).resolve()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def available_mean(rows: list[dict[str, Any]], field: str) -> float | None:
    values = [float(row[field]) for row in rows if row.get(field) is not None]
    return mean(values) if values else None


def category_macro_topology(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "category_count": len(rows),
        "sample_count_total": sum(int(row["sample_count"]) for row in rows),
        "unique_signature_count_mean": available_mean(rows, "unique_signature_count"),
        "unique_signature_rate_mean": available_mean(rows, "unique_signature_rate"),
        "mode_rate_mean": available_mean(rows, "mode_rate"),
        "pairwise_exact_rate_mean": available_mean(rows, "pairwise_exact_rate"),
        "normalized_entropy_mean": available_mean(rows, "normalized_entropy"),
        "aggregation": "unweighted mean over the five common categories",
        "claim": "name-free rooted topology diversity only; not semantic correctness",
    }


def scope_metrics(
    label: str,
    categories: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    requested_count: int,
) -> dict[str, Any]:
    aggregate = shared.aggregate(rows, requested_count=requested_count)
    per_category: list[dict[str, Any]] = []
    for category in categories:
        category_rows = [row for row in rows if row["factory"] == category["factory"]]
        category_requested = 6 if label == "paper_main30" else 36
        per_category.append(
            {
                "category_id": category["category_id"],
                "common_category": category["common_category"],
                "factory": category["factory"],
                "selection_reason": category["selection_reason"],
                "availability": {
                    "requested_count": category_requested,
                    "evaluated_count": len(category_rows),
                    "missing_count": category_requested - len(category_rows),
                    "evaluation_coverage": len(category_rows) / category_requested,
                },
                "aggregate": shared.aggregate(
                    category_rows, requested_count=category_requested
                ),
                "topology_consistency": shared.topology_consistency(category_rows),
            }
        )
    topology_rows = [row["topology_consistency"] for row in per_category]
    return {
        "label": label,
        "requested_count": requested_count,
        "evaluated_count": len(rows),
        "evaluation_coverage": len(rows) / requested_count,
        "aggregate": aggregate,
        "per_category": per_category,
        "topology_category_macro": category_macro_topology(topology_rows),
        "topology_aggregation_note": (
            "Signatures are compared only within category. Cross-category exact "
            "matching is not meaningful and is not pooled."
        ),
    }


def gold_audit(
    protocol: dict[str, Any], main_count: int, supplement_count: int
) -> dict[str, Any]:
    contract = protocol["independent_hierarchy_gold"]
    if contract["manifest"] is not None:
        raise NotImplementedError(
            "gold is configured but no independently frozen scorer is available"
        )
    return {
        "status": "BLOCKED_NO_INDEPENDENT_GOLD",
        "configured_gold_manifest": None,
        "paper_main_gold_coverage": {"eligible_assets": 0, "requested_assets": main_count},
        "supplement_gold_coverage": {
            "eligible_assets": 0,
            "requested_assets": supplement_count,
        },
        "parent_child_edge_f1": None,
        "hierarchy_exact_match": None,
        "semantic_nesting_accuracy": None,
        "requirements": {
            "parent_child_edge_f1": contract["required_for_edge_f1"],
            "hierarchy_exact_match": contract["required_for_exact_match"],
            "semantic_nesting_accuracy": contract["required_for_semantic_nesting"],
        },
        "rejected_as_self_ground_truth": contract["rejected_as_gold"],
        "conclusion": (
            "Validity, structural statistics, and topology stability are output-derived "
            "descriptors. They cannot establish hierarchy correctness without an "
            "independent asset-aligned gold tree and node-matching policy."
        ),
    }


def fmt(value: Any) -> str:
    return "N/A" if value is None else f"{float(value):.3f}"


def add_scope_report(lines: list[str], title: str, scope: dict[str, Any]) -> None:
    aggregate = scope["aggregate"]
    lines.extend(
        [
            f"## {title}",
            "",
            (
                f"- Evaluated: {scope['evaluated_count']}/{scope['requested_count']} "
                f"({scope['evaluation_coverage']:.3%}); Valid Tree: "
                f"{aggregate['valid_tree_count']}/{scope['requested_count']} "
                f"({aggregate['valid_tree_rate_requested']:.3%})"
            ),
            (
                "- Mean nodes/edges/leaves/internal/branching: "
                f"{fmt(aggregate['node_count_mean'])} / {fmt(aggregate['edge_count_mean'])} / "
                f"{fmt(aggregate['leaf_count_mean'])} / "
                f"{fmt(aggregate['internal_node_count_mean'])} / "
                f"{fmt(aggregate['branching_node_count_mean'])}"
            ),
            (
                "- Branching: mean internal out-degree "
                f"{fmt(aggregate['mean_internal_out_degree_mean'])}; max out-degree mean "
                f"{fmt(aggregate['max_out_degree_mean'])}; leaf ratio "
                f"{fmt(aggregate['leaf_ratio_mean'])}"
            ),
            (
                "- Mean movable/fixed edges and movable ratio: "
                f"{fmt(aggregate['movable_edge_count_mean'])} / "
                f"{fmt(aggregate['fixed_edge_count_mean'])} / "
                f"{fmt(aggregate['movable_edge_ratio_mean'])}"
            ),
            (
                "- Mean visual/collision/group link ratios: "
                f"{fmt(aggregate['visual_link_ratio_mean'])} / "
                f"{fmt(aggregate['collision_link_ratio_mean'])} / "
                f"{fmt(aggregate['group_node_ratio_mean'])}"
            ),
            (
                "- Root/component/cycle/malformed/multi-parent defects among evaluated: "
                f"{aggregate['root_defect_count']} / "
                f"{aggregate['component_defect_count']} / "
                f"{aggregate['cycle_defect_count']} / "
                f"{aggregate['malformed_edge_asset_count']} / "
                f"{aggregate['multi_parent_asset_count']}"
            ),
            (
                "- Mean largest-component and valid-joint-endpoint coverage: "
                f"{fmt(aggregate['largest_component_node_rate_mean_evaluated'])} / "
                f"{fmt(aggregate['valid_joint_endpoint_rate_mean_evaluated'])}"
            ),
            "",
            "| Category | Eval. | Nodes | Leaves | Branch nodes | Movable | Fixed | Visual cov. | Root defects | Unique topo. | Mode | Pair exact | Entropy |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for item in scope["per_category"]:
        metrics = item["aggregate"]
        topology = item["topology_consistency"]
        availability = item["availability"]
        lines.append(
            "| {category} | {evaluated}/{requested} | {nodes} | {leaves} | "
            "{branches} | {movable} | {fixed} | {visual} | {root_defects} | "
            "{unique} | {mode} | {pair} | {entropy} |".format(
                category=item["common_category"],
                evaluated=availability["evaluated_count"],
                requested=availability["requested_count"],
                nodes=fmt(metrics["node_count_mean"]),
                leaves=fmt(metrics["leaf_count_mean"]),
                branches=fmt(metrics["branching_node_count_mean"]),
                movable=fmt(metrics["movable_edge_count_mean"]),
                fixed=fmt(metrics["fixed_edge_count_mean"]),
                visual=fmt(metrics["visual_link_ratio_mean"]),
                root_defects=metrics["root_defect_count"],
                unique=topology["unique_signature_count"],
                mode=fmt(topology["mode_rate"]),
                pair=fmt(topology["pairwise_exact_rate"]),
                entropy=fmt(topology["normalized_entropy"]),
            )
        )
    topology = scope["topology_category_macro"]
    lines.extend(
        [
            "",
            (
                "Category-macro topology unique-rate/mode/pair/entropy: "
                f"{fmt(topology['unique_signature_rate_mean'])} / "
                f"{fmt(topology['mode_rate_mean'])} / "
                f"{fmt(topology['pairwise_exact_rate_mean'])} / "
                f"{fmt(topology['normalized_entropy_mean'])}"
            ),
            "",
        ]
    )


def render_report(
    main: dict[str, Any], supplement: dict[str, Any], gold: dict[str, Any]
) -> str:
    lines = [
        "# Infinite Mobility extended hierarchy metrics",
        "",
        "Selections are unchanged: main30 uses five categories x seeds 0-5; "
        "supplement180 uses the same categories x seeds 0-35.",
        "",
        "All structural metrics use the shared `hierarchy_extended_metrics.py` evaluator. "
        "Topology is a name-free rooted signature and is compared only within category.",
        "",
    ]
    add_scope_report(lines, "Paper main30", main)
    add_scope_report(lines, "Full supplement180", supplement)
    lines.extend(
        [
            "## Independent-gold correctness",
            "",
            f"- Status: `{gold['status']}`",
            "- Independent gold coverage: 0/30 main; 0/180 supplement",
            "- Parent-Child Edge F1: N/A",
            "- Hierarchy Exact Match: N/A",
            "- Semantic Nesting Accuracy: N/A",
            "",
            "Generated URDFs, factory source, and output-derived signatures are explicitly "
            "rejected as self-ground-truth. An independent aligned gold tree plus frozen "
            "node matching/adjudication is required.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--source-protocol", type=Path, default=DEFAULT_SOURCE_PROTOCOL)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    workspace_root = EXP_ROOT.parent.resolve(strict=True)
    protocol_path = hierarchy.ensure_contained(args.protocol, workspace_root)
    source_protocol_path = hierarchy.ensure_contained(
        args.source_protocol, workspace_root
    )
    input_root = hierarchy.ensure_contained(args.input_root, workspace_root)
    cohort_manifest_path = hierarchy.ensure_contained(
        args.cohort_manifest, workspace_root
    )
    output_root = hierarchy.ensure_contained(
        args.output_root, workspace_root, must_exist=False
    )
    hierarchy.ensure_contained(SHARED_EVALUATOR, workspace_root)

    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    source_protocol, _, source_records = hierarchy.load_and_validate_inputs(
        source_protocol_path, input_root
    )
    if source_protocol["protocol_id"] != protocol["source_seed_protocol"]:
        raise ValueError("source protocol mismatch")
    frozen_manifest = json.loads(cohort_manifest_path.read_text(encoding="utf-8"))
    frozen_selection = frozen_manifest["selection"]
    if len(frozen_selection) != 180:
        raise ValueError("frozen cohort must contain exactly 180 selections")

    record_map = {
        (str(row["factory"]), int(row["seed"])): row for row in source_records
    }
    category_map = {row["factory"]: row for row in protocol["categories"]}
    all_selection_records: list[dict[str, Any]] = []
    evaluated: list[dict[str, Any]] = []
    for frozen in frozen_selection:
        key = (str(frozen["factory"]), int(frozen["seed"]))
        category = category_map[key[0]]
        validation = hierarchy.evaluate_case(input_root, record_map[key])
        if validation["baseline_status"] != frozen["baseline_status"]:
            raise ValueError(f"frozen status mismatch for {key}")
        if validation.get("urdf_sha256") != frozen.get("urdf_sha256"):
            raise ValueError(f"frozen URDF hash mismatch for {key}")
        selection_record = {
            "category_id": category["category_id"],
            "common_category": category["common_category"],
            "factory": key[0],
            "seed": key[1],
            "paper_main": bool(frozen["paper_main"]),
            "baseline_status": validation["baseline_status"],
            "evaluation_status": validation["evaluation_status"],
            "urdf_path": validation.get("urdf_path"),
            "urdf_sha256": validation.get("urdf_sha256"),
        }
        all_selection_records.append(selection_record)
        if validation["evaluation_status"] != "PASS":
            continue
        urdf_path = hierarchy.ensure_contained(
            input_root / str(validation["urdf_path"]), input_root
        )
        metrics = shared.analyze_urdf(urdf_path)
        if bool(metrics["valid_tree"]) != bool(validation["valid_tree"]):
            raise ValueError(f"shared valid-tree result disagrees for {key}")
        evaluated.append({**selection_record, **metrics})

    main_rows = [row for row in evaluated if row["paper_main"]]
    main = scope_metrics("paper_main30", protocol["categories"], main_rows, 30)
    supplement = scope_metrics(
        "full_36_seed_supplement180", protocol["categories"], evaluated, 180
    )
    gold = gold_audit(protocol, 30, 180)
    summary = {
        "protocol_id": "nano3d_infinite_mobility_hierarchy_extended_shared_v1",
        "generated_at": utc_now(),
        "selection_unchanged": True,
        "shared_evaluator": {
            "path": SHARED_EVALUATOR.relative_to(EXP_ROOT).as_posix(),
            "sha256": sha256(SHARED_EVALUATOR),
            "apis": ["analyze_urdf", "aggregate", "topology_consistency"],
        },
        "definitions": {
            "node": "named URDF link",
            "edge": "URDF joint with valid parent and child endpoints",
            "leaf": "node with no children",
            "branching_node": "node with at least two children",
            "coverage": "evaluated generated assets divided by unchanged requested selection",
            "topology": "name-free rooted shape including joint type and visual/group node kind",
        },
        "paper_main30": main,
        "supplement180": supplement,
        "independent_gold_audit": gold,
    }
    manifest = {
        "protocol_sha256": sha256(protocol_path),
        "source_protocol_sha256": sha256(source_protocol_path),
        "frozen_cohort_manifest_sha256": sha256(cohort_manifest_path),
        "shared_evaluator_sha256": sha256(SHARED_EVALUATOR),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "selection_count": len(all_selection_records),
        "paper_main_selection_count": sum(
            bool(row["paper_main"]) for row in all_selection_records
        ),
        "evaluated_count": len(evaluated),
        "paper_main_evaluated_count": len(main_rows),
        "selection_unchanged": True,
        "selection": all_selection_records,
    }

    output_root.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Any] = {
        "extended_summary.json": summary,
        "extended_manifest.json": manifest,
        "extended_asset_records.json": evaluated,
        "extended_category_metrics.json": {
            "paper_main30": main["per_category"],
            "supplement180": supplement["per_category"],
        },
        "gold_availability.json": gold,
    }
    for name, payload in outputs.items():
        (output_root / name).write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    (output_root / "extended_report.md").write_text(
        render_report(main, supplement, gold), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"outputs={output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
