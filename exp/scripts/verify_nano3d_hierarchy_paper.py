#!/usr/bin/env python3
"""Verify the frozen Table 3 matched-category summaries and selections."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from math import isclose
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
RESULT_ROOT = EXP_ROOT / "runtime/nano3d_hierarchy_paper"
EXPECTED_CATEGORIES = {
    "storage_furniture_cabinet",
    "table",
    "refrigerator",
    "dishwasher",
    "oven",
}


def load_json(relative: str) -> Any:
    return json.loads((RESULT_ROOT / relative).read_text(encoding="utf-8"))


def load_jsonl(relative: str) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in (RESULT_ROOT / relative).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if isinstance(actual, float) or isinstance(expected, float):
        if not isclose(float(actual), float(expected), rel_tol=0, abs_tol=1e-12):
            raise AssertionError(f"{label}: {actual!r} != {expected!r}")
    elif actual != expected:
        raise AssertionError(f"{label}: {actual!r} != {expected!r}")


def assert_six_per_category(rows: list[dict[str, Any]], field: str, label: str) -> None:
    counts = Counter(str(row[field]) for row in rows)
    if set(counts) != EXPECTED_CATEGORIES or set(counts.values()) != {6}:
        raise AssertionError(f"{label}: expected five categories x six, found {counts}")


def main() -> int:
    combined = load_json("summary.json")
    pva = load_json("pva/summary.json")
    lam = load_json("lam/summary.json")
    articraft = load_json("articraft/summary.json")
    mobility = load_json("infinite_mobility/summary.json")

    sources = {
        "pva": {
            "requested_count": pva["overall"]["selected_count"],
            "valid_tree_count": pva["overall"]["valid_tree_count"],
            "has_hierarchy_count": pva["overall"]["has_hierarchy_count"],
            "semantic_depth_mean": pva["overall"]["semantic_depth_mean"],
            "named_groups_mean": pva["overall"]["named_groups_mean"],
            "pivots_mean": pva["overall"]["pivots_mean"],
        },
        "lam": {
            "requested_count": lam["selected_count"],
            "valid_tree_count": lam["metrics"]["valid_tree_count"],
            "has_hierarchy_count": lam["metrics"]["has_hierarchy_count"],
            "semantic_depth_mean": lam["metrics"]["semantic_depth_mean"],
            "named_groups_mean": lam["metrics"]["named_groups_mean"],
            "pivots_mean": lam["metrics"]["pivots_mean"],
        },
        "articraft": {
            "requested_count": articraft["selected_cohort"]["selected_count"],
            "valid_tree_count": articraft["hierarchy_metrics"]["valid_tree_count"],
            "has_hierarchy_count": articraft["hierarchy_metrics"]["has_tree_count"],
            "semantic_depth_mean": articraft["hierarchy_metrics"]["semantic_depth_mean"],
            "named_groups_mean": articraft["hierarchy_metrics"]["named_groups_mean"],
            "pivots_mean": articraft["hierarchy_metrics"]["pivots_mean"],
        },
        "infinite_mobility": {
            "requested_count": mobility["paper_main"]["micro"]["requested_count"],
            "valid_tree_count": mobility["paper_main"]["micro"]["valid_tree_count"],
            "has_hierarchy_count": mobility["paper_main"]["micro"]["has_hierarchy_count"],
            "semantic_depth_mean": mobility["paper_main"]["micro"]["semantic_depth_mean"],
            "named_groups_mean": mobility["paper_main"]["micro"]["named_groups_mean"],
            "pivots_mean": mobility["paper_main"]["micro"]["pivots_mean"],
        },
    }
    for method, fields in sources.items():
        for field, source_value in fields.items():
            assert_equal(combined["metrics"][method][field], source_value, f"{method}.{field}")

    pva_manifest = load_json("pva/frozen_manifest.json")["selection"]
    for row in pva_manifest:
        if row["category"] == "storage_furniture":
            row["category"] = "storage_furniture_cabinet"
    assert_six_per_category(pva_manifest, "category", "PV-A selection")
    assert_six_per_category(load_jsonl("lam/manifest.jsonl"), "shared_class", "LAM selection")
    assert_six_per_category(
        load_jsonl("articraft/selection_manifest.jsonl"), "cohort", "Articraft selection"
    )
    mobility_main = [
        row
        for row in load_json("infinite_mobility/cohort_manifest.json")["selection"]
        if row["paper_main"]
    ]
    assert_six_per_category(mobility_main, "category_id", "Infinite Mobility selection")

    if articraft["failure_counts"]:
        raise AssertionError(f"Articraft reports failures: {articraft['failure_counts']}")
    if lam["selection"]["selected_tier_counts"] != {
        "broken": 2,
        "loads_only": 4,
        "viable": 24,
    }:
        raise AssertionError("LAM selected-tier audit changed")
    if mobility["full_36_seed_supplement"] != {
        "generated_asset_count": 175,
        "has_hierarchy_count": 175,
        "requested_count": 180,
        "valid_tree_count": 175,
    }:
        raise AssertionError("Infinite Mobility 180-case supplement changed")

    pva_extended = load_json("pva/extended/summary.json")
    lam_extended = load_json("lam/extended_summary.json")
    articraft_extended = load_json("articraft/extended_summary.json")
    mobility_extended = load_json("infinite_mobility/extended_summary.json")
    shared_evaluator = EXP_ROOT / "scripts/hierarchy_extended_metrics.py"
    assert_equal(
        combined["extended_metrics"]["shared_evaluator_sha256"],
        sha256(shared_evaluator),
        "extended shared evaluator hash",
    )
    extended_sources = {
        "pva": (
            pva_extended["overall"],
            pva_extended["category_macro_topology_consistency"],
            {
                "topology_unique_signature_rate_macro": "unique_signature_rate",
                "topology_mode_rate_macro": "mode_rate",
                "topology_pairwise_exact_rate_macro": "pairwise_exact_rate",
                "topology_normalized_entropy_macro": "normalized_entropy",
            },
        ),
        "lam": (
            lam_extended["overall"],
            lam_extended["category_normalized_topology_consistency"]["macro"],
            {
                "topology_unique_signature_rate_macro": "unique_signature_rate_macro",
                "topology_mode_rate_macro": "mode_rate_macro",
                "topology_pairwise_exact_rate_macro": "pairwise_exact_rate_macro",
                "topology_normalized_entropy_macro": "normalized_entropy_macro",
            },
        ),
        "articraft": (
            articraft_extended["overall"],
            articraft_extended["topology_consistency"]["category_macro"],
            {
                "topology_unique_signature_rate_macro": "unique_signature_rate_macro",
                "topology_mode_rate_macro": "mode_rate_macro",
                "topology_pairwise_exact_rate_macro": "pairwise_exact_rate_macro",
                "topology_normalized_entropy_macro": "normalized_entropy_macro",
            },
        ),
        "infinite_mobility": (
            mobility_extended["paper_main30"]["aggregate"],
            mobility_extended["paper_main30"]["topology_category_macro"],
            {
                "topology_unique_signature_rate_macro": "unique_signature_rate_mean",
                "topology_mode_rate_macro": "mode_rate_mean",
                "topology_pairwise_exact_rate_macro": "pairwise_exact_rate_mean",
                "topology_normalized_entropy_macro": "normalized_entropy_mean",
            },
        ),
    }
    aggregate_fields = {
        "evaluated_count",
        "node_count_mean",
        "edge_count_mean",
        "leaf_count_mean",
        "leaf_ratio_mean",
        "internal_node_count_mean",
        "branching_node_count_mean",
        "mean_internal_out_degree_mean",
        "max_out_degree_mean",
        "movable_depth_mean",
        "movable_edge_count_mean",
        "fixed_edge_count_mean",
        "movable_edge_ratio_mean",
        "visual_link_ratio_mean",
        "collision_link_ratio_mean",
        "root_defect_count",
        "component_defect_count",
        "malformed_edge_asset_count",
        "multi_parent_asset_count",
    }
    for method, (source_aggregate, source_topology, topology_fields) in extended_sources.items():
        combined_metrics = combined["extended_metrics"]["table_metrics"][method]
        for field in aggregate_fields:
            assert_equal(combined_metrics[field], source_aggregate[field], f"extended {method}.{field}")
        for combined_field, source_field in topology_fields.items():
            assert_equal(
                combined_metrics[combined_field],
                source_topology[source_field],
                f"extended {method}.{combined_field}",
            )

    print("Table 3 matched-category verification: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
