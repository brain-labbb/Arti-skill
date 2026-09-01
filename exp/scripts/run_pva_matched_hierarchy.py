#!/usr/bin/env python3
"""Build and evaluate the frozen five-category PV-A hierarchy cohort."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import itertools
import json
import os
from pathlib import Path
from statistics import mean
import sys
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve(strict=True)
REPO_ROOT = (WORKSPACE / "arti-skill").resolve(strict=True)
TEMPLATE_ROOT = (REPO_ROOT / "arti-template").resolve(strict=True)
EXP_ROOT = (REPO_ROOT / "exp").resolve(strict=True)
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/pva"
SHARED_EVALUATOR = EXP_ROOT / "scripts/run_nano3d_hierarchy.py"
PROTOCOL_ID = "nano3d_hierarchy_matched_five_category_v1"
SEEDS = list(range(6))
CATEGORY_TEMPLATES = {
    "storage_furniture": "drawer_cabinet_with_sliding_drawers",
    "table": "folding_camp_table",
    "refrigerator": "refrigerator_with_hinged_doors",
    "dishwasher": "dishwasher_with_dropdown_door_and_sliding_racks",
    "oven": "Other_Built_in_oven",
}


def contained(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    resolved.relative_to(WORKSPACE)
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    contained(path.parent, must_exist=False).mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def mode_rate(values: list[object]) -> float | None:
    if not values:
        return None
    return Counter(values).most_common(1)[0][1] / len(values)


def pairwise_exact_rate(values: list[object]) -> float | None:
    pairs = [left == right for left, right in itertools.combinations(values, 2)]
    return mean(pairs) if pairs else None


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [row for row in rows if row.get("parseable_final_urdf")]
    return {
        "selected_count": len(rows),
        "compile_pass_count": sum(row["compile_status"] == "PASS" for row in rows),
        "parseable_final_urdf_count": len(valid),
        "valid_tree_count": sum(bool(row.get("valid_tree")) for row in rows),
        "has_hierarchy_count": sum(bool(row.get("has_tree")) for row in rows),
        "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in valid)
        if valid
        else None,
        "semantic_depth_min": min(int(row["semantic_depth"]) for row in valid)
        if valid
        else None,
        "semantic_depth_max": max(int(row["semantic_depth"]) for row in valid)
        if valid
        else None,
        "named_groups_mean": mean(int(row["named_group_count"]) for row in valid)
        if valid
        else None,
        "named_groups_total": sum(int(row["named_group_count"]) for row in valid),
        "pivots_mean": mean(int(row["pivot_count"]) for row in valid)
        if valid
        else None,
        "pivots_total": sum(int(row["pivot_count"]) for row in valid),
    }


def render_report(summary: dict[str, Any]) -> str:
    overall = summary["overall"]
    lines = [
        "# PV-A matched-category Table 3 Hierarchy",
        "",
        f"- Protocol: `{summary['protocol_id']}`",
        "- Cohort: 5 categories x seeds 0-5 = 30 requested assets",
        f"- Strict compile: {overall['compile_pass_count']}/{overall['selected_count']}",
        f"- Valid Tree: {overall['valid_tree_count']}/{overall['selected_count']}",
        f"- Has Hierarchy: {overall['has_hierarchy_count']}/{overall['selected_count']}",
        (
            "- Semantic Depth: "
            f"{overall['semantic_depth_mean']:.3f} mean "
            f"[{overall['semantic_depth_min']}, {overall['semantic_depth_max']}]"
        ),
        (
            f"- Named Groups: {overall['named_groups_mean']:.3f} mean; "
            f"{overall['named_groups_total']} total"
        ),
        f"- Pivots: {overall['pivots_mean']:.3f} mean; {overall['pivots_total']} total",
        "- Edge F1 / Exact Match / Semantic Nesting: N/A (no independent gold)",
        "",
        "## Per category",
        "",
        "| Category | Template | Valid Tree | Has Hierarchy | Depth | Groups | Pivots |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for category, metrics in summary["per_category"].items():
        lines.append(
            f"| {category} | `{CATEGORY_TEMPLATES[category]}` | "
            f"{metrics['valid_tree_count']}/{metrics['selected_count']} | "
            f"{metrics['has_hierarchy_count']}/{metrics['selected_count']} | "
            f"{metrics['semantic_depth_mean']:.3f} | "
            f"{metrics['named_groups_mean']:.3f} | {metrics['pivots_mean']:.3f} |"
        )
    lines.extend(
        [
            "",
            "Cross-seed metrics describe topology stability only; they do not score semantic correctness.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()

    output = contained(args.output, must_exist=False)
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(
        {
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
    )

    sys.path.insert(0, str(REPO_ROOT))
    sys.path.insert(0, str(TEMPLATE_ROOT))
    from agent.template_registry import TEMPLATE_REGISTRY
    from agent.template_sweep import run_seed_outcomes
    from exp.scripts.run_nano3d_hierarchy import parse_hierarchy

    selection: list[dict[str, Any]] = []
    template_rows: dict[str, dict[str, Any]] = {}
    for category, slug in CATEGORY_TEMPLATES.items():
        if slug not in TEMPLATE_REGISTRY:
            raise RuntimeError(f"template absent from registry: {slug}")
        template_path = contained(TEMPLATE_ROOT / "agent/templates" / f"{slug}.py")
        template_rows[category] = {
            "category": category,
            "slug": slug,
            "stem": TEMPLATE_REGISTRY[slug],
            "template_path": str(template_path),
            "template_sha256": sha256(template_path),
        }
        for seed in SEEDS:
            selection.append({"category": category, "slug": slug, "seed": seed})

    manifest = {
        "protocol_id": PROTOCOL_ID,
        "method": "PV-A",
        "selection_rule": "five frozen category-template mappings x seeds 0-5",
        "categories": list(CATEGORY_TEMPLATES),
        "seeds": SEEDS,
        "templates": list(template_rows.values()),
        "selection": selection,
        "shared_evaluator": str(SHARED_EVALUATOR),
        "shared_evaluator_sha256": sha256(SHARED_EVALUATOR),
        "strict_contract": {
            "target": "full",
            "author_tests": True,
            "compiler_baseline": True,
            "motion_qc": True,
            "hard_timeout_seconds": args.timeout,
        },
    }
    dump_json(output / "frozen_manifest.json", manifest)

    records: list[dict[str, Any]] = []
    for category, template in template_rows.items():
        artifact_root = contained(output / "packages" / category, must_exist=False)
        outcomes = run_seed_outcomes(
            slug=template["slug"],
            stem=template["stem"],
            seeds=SEEDS,
            sdk_package="sdk",
            max_workers=max(1, args.workers),
            repo_root=TEMPLATE_ROOT,
            compile_timeout_s=args.timeout,
            motion_qc=True,
            artifact_root=artifact_root,
        )
        for outcome in outcomes:
            row: dict[str, Any] = {
                "category": category,
                "slug": template["slug"],
                "seed": outcome.seed,
                "compile_status": "PASS" if outcome.verdict == "pass" else "FAIL",
                "compile_elapsed_seconds": outcome.elapsed_s,
                "failure_type": outcome.failure_type,
                "failure_details": outcome.failure_details,
                "config": outcome.config,
                "artifact_dir": outcome.artifact_dir,
                "parseable_final_urdf": False,
                "valid_tree": False,
                "has_tree": False,
            }
            if outcome.verdict == "pass" and outcome.artifact_dir:
                artifact_dir = contained(Path(outcome.artifact_dir))
                urdf = contained(artifact_dir / "model.urdf")
                hierarchy = parse_hierarchy(urdf)
                row.update(hierarchy)
                row.update(
                    {
                        "parseable_final_urdf": True,
                        "model_urdf": str(urdf),
                        "model_urdf_sha256": sha256(urdf),
                    }
                )
            records.append(row)

    records.sort(key=lambda row: (row["category"], row["seed"]))
    per_category = {
        category: summarize([row for row in records if row["category"] == category])
        for category in CATEGORY_TEMPLATES
    }
    cross_seed = {}
    for category in CATEGORY_TEMPLATES:
        valid = [
            row
            for row in records
            if row["category"] == category and row["parseable_final_urdf"]
        ]
        signatures = [
            tuple(tuple(edge) for edge in row["edge_signature"]) for row in valid
        ]
        cross_seed[category] = {
            "valid_asset_count": len(valid),
            "raw_edge_signature_mode_rate": mode_rate(signatures),
            "pairwise_raw_edge_exact_rate": pairwise_exact_rate(signatures),
        }

    summary = {
        "protocol_id": PROTOCOL_ID,
        "method": "PV-A",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "evaluation_unit": "one frozen template category and seed",
        "category_count": len(CATEGORY_TEMPLATES),
        "requested_asset_count": len(selection),
        "overall": summarize(records),
        "per_category": per_category,
        "cross_seed_supplement": cross_seed,
        "unsupported": {
            "parent_child_edge_f1": "no independent hierarchy gold",
            "hierarchy_exact_match": "no independent hierarchy gold",
            "semantic_nesting_accuracy": "no independent hierarchy gold",
        },
    }
    dump_json(output / "records.json", records)
    dump_json(output / "summary.json", summary)
    (output / "report.md").write_text(render_report(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["overall"]["compile_pass_count"] == len(selection) else 1


if __name__ == "__main__":
    raise SystemExit(main())
