#!/usr/bin/env python3
"""Aggregate the frozen four-method category-matched Table 2 Naming panel."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
PROTOCOL = PROJECT_ROOT / "exp/reference/table2_naming_matched_protocol_v1.json"
OUT = PROJECT_ROOT / "exp/runtime/table2_naming_matched_v1"
INPUTS = {
    "Ours": PROJECT_ROOT / "exp/runtime/table2_naming_matched_ours_v1/summary.json",
    "LAM": PROJECT_ROOT / "exp/runtime/lam_naming_matched_v1/summary.json",
    "Articraft": PROJECT_ROOT / "exp/runtime/articraft_naming_matched_v1/summary.json",
    "Infinite Mobility": PROJECT_ROOT
    / "exp/runtime/infinite_mobility_naming_v1/matched35/summary.json",
}
SELFCHECKS = {
    "Ours": PROJECT_ROOT
    / "exp/runtime/table2_naming_matched_ours_v1/reproducibility_check.json",
    "LAM": PROJECT_ROOT / "exp/runtime/lam_naming_matched_v1/self_check.json",
    "Articraft": PROJECT_ROOT / "exp/runtime/articraft_naming_matched_v1/self_check.json",
    "Infinite Mobility": PROJECT_ROOT
    / "exp/runtime/infinite_mobility_naming_v1/matched35/self_check.json",
}


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def digest(path: Path) -> str:
    target = contained(path)
    if target.is_symlink() or not target.is_file():
        raise RuntimeError(f"not a regular workspace file: {target}")
    return hashlib.sha256(target.read_bytes()).hexdigest()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(contained(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"expected object: {path}")
    return value


def write(path: Path, content: str) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(target)


def row_ours(value: dict[str, Any]) -> dict[str, Any]:
    direct = value["direct_metrics"]
    cross = value["cross_seed"]
    return {
        "method": "Ours",
        "source_scope": "reusable frozen templates, seeds 0..6; visual compile",
        "assets": value["coverage"]["naming_evaluable_assets"],
        "parts_total": direct["total_renderable_parts"],
        "parts_per_asset_mean": direct["parts_per_asset_mean"],
        "parts_per_asset_95ci": direct["category_cluster_bootstrap_95ci"][
            "parts_per_asset_mean"
        ],
        "named_parts": direct["total_named_renderable_parts"],
        "nameability_micro": direct["nameability_micro"],
        "opaque_generated_index_name_rate": None,
        "cross_seed": {
            "status": "measured raw-name proxy",
            "set_jaccard_pair_micro": cross["raw_unique_name_set_jaccard_pair_micro"],
            "set_jaccard_category_macro": cross[
                "raw_unique_name_set_jaccard_category_macro"
            ],
            "category_macro_95ci": None,
            "exact_multiset_mode_category_macro": cross[
                "exact_raw_name_multiset_mode_rate_category_macro"
            ],
        },
        "semantic_metrics": value["semantic_metrics"],
    }


def row_lam(value: dict[str, Any]) -> dict[str, Any]:
    direct = value["direct_metrics"]
    return {
        "method": "LAM",
        "source_scope": "deterministic official-release resample; not same-prompt or same-seed",
        "assets": direct["assets"],
        "parts_total": direct["total_renderable_parts"],
        "parts_per_asset_mean": direct["parts_per_asset_mean"],
        "parts_per_asset_95ci": value["bootstrap_ci"]["parts_per_asset_mean"],
        "named_parts": direct["total_named_renderable_parts"],
        "nameability_micro": direct["nameability_micro"],
        "opaque_generated_index_name_rate": None,
        "cross_seed": {"status": "N/A: no reusable release seed identity"},
        "semantic_metrics": value["semantic_metrics"],
    }


def row_articraft(value: dict[str, Any]) -> dict[str, Any]:
    direct = value["direct_metrics"]
    return {
        "method": "Articraft",
        "source_scope": "deterministic official-release resample; not same-prompt or same-seed",
        "assets": direct["assets"],
        "parts_total": direct["total_renderable_parts"],
        "parts_per_asset_mean": direct["parts_per_asset_mean"],
        "parts_per_asset_95ci": value["bootstrap_ci"]["parts_per_asset_mean"],
        "named_parts": direct["total_named_renderable_parts"],
        "nameability_micro": direct["nameability_micro"],
        "opaque_generated_index_name_rate": None,
        "cross_seed": {"status": "N/A: no reusable release seed identity"},
        "semantic_metrics": value["semantic_metrics"],
    }


def row_infinite(value: dict[str, Any]) -> dict[str, Any]:
    return {
        "method": "Infinite Mobility",
        "source_scope": "official reusable factories, seeds 0..6",
        "assets": value["asset_count"],
        "parts_total": value["urdf_part_node_total"],
        "parts_per_asset_mean": value["parts_per_asset_mean"],
        "parts_per_asset_95ci": value[
            "parts_per_asset_category_cluster_bootstrap_95ci"
        ],
        "named_parts": value["named_urdf_part_node_total"],
        "nameability_micro": value["nameability_micro"],
        "opaque_generated_index_name_rate": value[
            "opaque_generated_index_name_rate_micro"
        ],
        "cross_seed": {
            "status": "measured raw-name proxy",
            "set_jaccard_pair_micro": value["raw_unique_name_set_jaccard_pair_micro"],
            "set_jaccard_category_macro": value[
                "raw_unique_name_set_jaccard_category_macro"
            ],
            "category_macro_95ci": value[
                "raw_unique_name_set_jaccard_category_macro_95ci"
            ],
            "exact_multiset_mode_category_macro": value[
                "exact_raw_name_multiset_mode_rate_category_macro"
            ],
        },
        "semantic_metrics": {
            "semantic_precision": value["semantic_precision"],
            "semantic_recall": value["semantic_recall"],
            "naming_richness": value["naming_richness"],
            "functional_core_coverage": value["functional_core_coverage"],
            "instance_discriminability": value["instance_discriminability"],
            "over_segmentation_rate": value["over_segmentation_rate"],
        },
    }


def report(payload: dict[str, Any]) -> str:
    lines = [
        "# Table 2 Naming: category-matched four-method panel",
        "",
        "Five categories x seven assets per category per method; 140/140 assets evaluable.",
        "This is category- and N-matched, not same-prompt.",
        "",
        "| Method | Parts/asset [95% category-cluster CI] | Named | Cross-seed |",
        "|---|---:|---:|---|",
    ]
    for row in payload["methods"]:
        ci = row["parts_per_asset_95ci"]
        cross = row["cross_seed"]
        cross_text = cross["status"]
        if cross_text == "measured raw-name proxy":
            cross_text = (
                f"raw set Jaccard {cross['set_jaccard_pair_micro']:.3f}; "
                f"exact mode {cross['exact_multiset_mode_category_macro']:.3f}"
            )
        lines.append(
            f"| {row['method']} | {row['parts_per_asset_mean']:.3f} "
            f"[{ci[0]:.3f}, {ci[1]:.3f}] | {row['named_parts']}/{row['parts_total']} "
            f"= {row['nameability_micro']:.3f} | {cross_text} |"
        )
    lines.extend(
        [
            "",
            "Infinite Mobility's lexical names are all opaque l_<index>; its 1.0 Nameability is not semantic naming evidence.",
            "All six semantic fields remain N/A because output-independent gold and three blind judge verdicts are absent.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    protocol = load(PROTOCOL)
    protocol_hash = digest(PROTOCOL)
    values = {method: load(path) for method, path in INPUTS.items()}
    rows = [
        row_ours(values["Ours"]),
        row_lam(values["LAM"]),
        row_articraft(values["Articraft"]),
        row_infinite(values["Infinite Mobility"]),
    ]
    self_checks = {method: load(path) for method, path in SELFCHECKS.items()}
    payload = {
        "protocol_id": protocol["protocol_id"],
        "protocol_sha256": protocol_hash,
        "comparison_scope": {
            "category_matched": True,
            "equal_n": True,
            "same_prompt": False,
            "same_seed_semantics": False,
            "categories": protocol["design"]["canonical_categories"],
            "assets_per_category_per_method": 7,
            "assets_per_method": 35,
            "total_assets": 140,
        },
        "methods": rows,
        "input_summary_sha256": {
            method: digest(path) for method, path in INPUTS.items()
        },
        "input_self_check_sha256": {
            method: digest(path) for method, path in SELFCHECKS.items()
        },
    }
    checks = {
        "protocol_hash_expected": protocol_hash
        == "f4c4d915ce2e1bed5c99efd9dcebb97526a7d2639ad5021e3b068c3cf6105113",
        "method_order_exact": [row["method"] for row in rows]
        == ["Ours", "LAM", "Articraft", "Infinite Mobility"],
        "all_methods_35_assets": all(row["assets"] == 35 for row in rows),
        "total_assets_140": sum(row["assets"] for row in rows) == 140,
        "all_direct_counts_conserved": all(
            row["named_parts"] <= row["parts_total"]
            and row["parts_total"] / row["assets"] == row["parts_per_asset_mean"]
            for row in rows
        ),
        "all_semantic_metrics_null": all(
            value is None
            for row in rows
            for key, value in row["semantic_metrics"].items()
            if key != "reason"
        ),
        "all_input_self_checks_pass": (
            self_checks["Ours"]["status"] == "PASS"
            and self_checks["LAM"]["status"] == "PASS"
            and self_checks["Articraft"]["status"] == "PASS"
            and self_checks["Infinite Mobility"]["all_checks_pass"] is True
        ),
        "im_opaque_disclosure_present": rows[3]["opaque_generated_index_name_rate"]
        == 1.0,
        "comparison_not_mislabeled": payload["comparison_scope"]["same_prompt"] is False,
    }
    self_check = {
        "protocol_id": "nano3d_table2_naming_matched_aggregate_self_check_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "aggregator_sha256": digest(Path(__file__)),
    }
    write(OUT / "summary.json", json.dumps(payload, indent=2, sort_keys=True) + "\n")
    write(OUT / "report.md", report(payload))
    write(OUT / "self_check.json", json.dumps(self_check, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status": self_check["status"], "checks": len(checks)}))
    return 0 if self_check["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
