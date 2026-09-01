#!/usr/bin/env python3
"""Extended shared-metric hierarchy audit for the frozen Articraft cohort."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
from statistics import mean
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTI_SKILL = WORKSPACE / "arti-skill"
DEFAULT_INPUT = ARTI_SKILL / "exp/runtime/nano3d_hierarchy_paper/articraft"
DEFAULT_OUTPUT = DEFAULT_INPUT
DATASET = ARTI_SKILL / "exp/baselines/Articraft-10K-official"
PAPER_HARNESS = ARTI_SKILL / "exp/baselines/Articraft-paper-harness"
CURRENT_REPO = ARTI_SKILL / "exp/baselines/Articraft-official"
SHARED_EVALUATOR = ARTI_SKILL / "exp/scripts/hierarchy_extended_metrics.py"
FROZEN_SELECTION_SHA256 = "dea74ac6b6007a9c63be914de814c907f60e96ca0fa44ba03d654481536f732b"
EXPECTED_COHORTS = {
    "storage_furniture_cabinet": 6,
    "table": 6,
    "refrigerator": 6,
    "dishwasher": 6,
    "oven": 6,
}
PROTOCOL = "articraft_10k_frozen30_shared_extended_hierarchy_v1"


def contained(path: Path) -> Path:
    resolved = path.resolve(strict=False)
    if not resolved.is_relative_to(WORKSPACE):
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def load_shared_evaluator() -> tuple[Any, Any, Any, str]:
    evaluator = contained(SHARED_EVALUATOR)
    spec = importlib.util.spec_from_file_location("shared_hierarchy_extended", evaluator)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load shared evaluator: {evaluator}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in ("analyze_urdf", "aggregate", "topology_consistency"):
        if not callable(getattr(module, name, None)):
            raise RuntimeError(f"shared evaluator is missing callable {name}")
    return module.analyze_urdf, module.aggregate, module.topology_consistency, sha256(evaluator)


def finite_mean(values: list[float | None]) -> float | None:
    present = [float(value) for value in values if value is not None]
    return mean(present) if present else None


def metric_ranges(rows: list[dict[str, Any]]) -> dict[str, dict[str, float | None]]:
    fields = (
        "node_count",
        "edge_count",
        "leaf_count",
        "leaf_ratio",
        "branching_node_count",
        "mean_internal_out_degree",
        "max_out_degree",
        "movable_edge_count",
        "fixed_edge_count",
        "movable_edge_ratio",
        "visual_link_ratio",
        "collision_link_ratio",
        "largest_component_node_rate",
        "valid_joint_endpoint_rate",
    )
    valid = [row for row in rows if row["valid_tree"]]
    result: dict[str, dict[str, float | None]] = {}
    for field in fields:
        values = [float(row[field]) for row in valid if row[field] is not None]
        result[field] = {
            "min": min(values) if values else None,
            "max": max(values) if values else None,
        }
    return result


def category_macro_topology(per_category: dict[str, dict[str, Any]]) -> dict[str, Any]:
    fields = (
        "unique_signature_rate",
        "mode_rate",
        "pairwise_exact_rate",
        "normalized_entropy",
    )
    return {
        "category_count": len(per_category),
        "equal_category_weighting": True,
        **{
            f"{field}_macro": finite_mean([row[field] for row in per_category.values()])
            for field in fields
        },
        "claim": (
            "equal-category macro of name-free rooted topology consistency; "
            "not gold-referenced semantic correctness"
        ),
    }


def independent_gold_audit(input_dir: Path) -> dict[str, Any]:
    pattern = (
        r"gold[_ -]?(hierarchy|edge|tree)|"
        r"ground[_ -]?truth.*(hierarchy|parent|edge|tree)|"
        r"expected[_ -]?(hierarchy|parent|edge|tree)|"
        r"hierarchy[_ -]?(annotation|label)|"
        r"parent[_ -]?child[_ -]?(annotation|label)|"
        r"nesting[_ -]?(annotation|label)"
    )
    search_paths = [contained(DATASET), contained(PAPER_HARNESS), contained(CURRENT_REPO)]
    command = [
        "rg",
        "-n",
        "-i",
        "--glob",
        "*.json",
        "--glob",
        "*.jsonl",
        "--glob",
        "*.md",
        "--glob",
        "!uv.lock",
        pattern,
        *[str(path) for path in search_paths],
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode not in (0, 1):
        raise RuntimeError(f"independent-gold search failed: {completed.stderr.strip()}")
    matches = [line for line in completed.stdout.splitlines() if line.strip()]
    selected_records = contained(input_dir / "selected_data/records")
    selected_artifact_names = sorted(
        {path.name for path in selected_records.rglob("*") if path.is_file()}
    )
    independent_gold_found = bool(matches)
    return {
        "independent_gold_found": independent_gold_found,
        "repository_search_match_count": len(matches),
        "repository_search_matches": matches[:100],
        "search_pattern": pattern,
        "searched_paths": [str(path) for path in search_paths],
        "selected_release_artifact_names": selected_artifact_names,
        "assessment": (
            "candidate structured-gold references require manual validation"
            if independent_gold_found
            else "no independent structured hierarchy gold found in the official release or code repositories"
        ),
        "disallowed_as_gold": [
            "materialized model.urdf: evaluated output",
            "released model.py: generator source for the evaluated output",
            "compile_report.json: validation derived from the same output",
        ],
        "insufficient_for_structured_gold": [
            "prompt.txt: free-form generation request, not a complete edge annotation",
            "record rating: scalar release-quality judgment, not a hierarchy annotation",
        ],
        "metric_status": {
            "parent_child_edge_f1": "N/A: no independent parent-child edge annotation",
            "hierarchy_exact_match": "N/A: no independent complete hierarchy annotation",
            "semantic_nesting_accuracy": "N/A: no independent semantic-nesting labels",
        },
    }


def fmt(value: float | None) -> str:
    return "N/A" if value is None else f"{value:.3f}"


def build_report(summary: dict[str, Any]) -> str:
    overall = summary["overall"]
    ranges = summary["overall_valid_tree_ranges"]
    topology = summary["topology_consistency"]
    macro = topology["category_macro"]
    gold = summary["independent_gold_audit"]

    def mean_range(field: str) -> str:
        return (
            f"{overall[field + '_mean']:.3f} "
            f"[{ranges[field]['min']:.3f}, {ranges[field]['max']:.3f}]"
        )

    lines = [
        "# Articraft frozen-30 extended hierarchy audit",
        "",
        "Status: **COMPLETE**",
        "",
        "This report re-parses the unchanged frozen 30 official-release URDFs with the shared extended hierarchy evaluator. Metrics are structural descriptors unless explicitly stated otherwise.",
        "",
        "## Frozen cohort integrity",
        "",
        f"- Selection N: {summary['frozen_cohort']['record_count']} (6 per category)",
        f"- Selection SHA-256: `{summary['frozen_cohort']['selection_manifest_sha256']}`",
        f"- Verified URDF content hashes: {summary['frozen_cohort']['urdf_hash_verified_count']}/30",
        "- Selection membership, rank, denominator, and failure-retention rule are unchanged.",
        "",
        "## Overall extended metrics",
        "",
        "| Metric | Result |",
        "|---|---:|",
        f"| Valid rooted tree | {overall['valid_tree_count']}/{overall['requested_count']} |",
        f"| Nodes per asset | {mean_range('node_count')} |",
        f"| Edges per asset | {mean_range('edge_count')} |",
        f"| Leaves per asset | {mean_range('leaf_count')} |",
        f"| Leaf ratio | {mean_range('leaf_ratio')} |",
        f"| Branch nodes per asset | {mean_range('branching_node_count')} |",
        f"| Internal-node out-degree | {mean_range('mean_internal_out_degree')} |",
        f"| Maximum out-degree | {mean_range('max_out_degree')} |",
        f"| Movable edges per asset | {mean_range('movable_edge_count')} |",
        f"| Fixed edges per asset | {mean_range('fixed_edge_count')} |",
        f"| Movable-edge ratio | {mean_range('movable_edge_ratio')} |",
        f"| Visual-link coverage | {mean_range('visual_link_ratio')} |",
        f"| Collision-link coverage | {mean_range('collision_link_ratio')} |",
        f"| Largest-component node coverage | {overall['largest_component_node_rate_mean_evaluated']:.3f} |",
        f"| Valid joint-endpoint coverage | {overall['valid_joint_endpoint_rate_mean_evaluated']:.3f} |",
        "",
        "## Root and graph defects",
        "",
        f"- Single-root: {overall['single_root_count']}/{overall['evaluated_count']}; root defects: {overall['root_defect_count']}",
        f"- Connected: {overall['connected_count']}/{overall['evaluated_count']}; component defects: {overall['component_defect_count']}",
        f"- Cycle defects: {overall['cycle_defect_count']}; malformed-edge assets: {overall['malformed_edge_asset_count']}",
        f"- Multi-parent assets: {overall['multi_parent_asset_count']}; unnamed/duplicate-link assets: {overall['unnamed_or_duplicate_link_asset_count']}",
        "",
        "## Per-category and category-macro topology consistency",
        "",
        "The shared evaluator canonicalizes rooted topology without link names, while retaining joint types and visual/group node roles. These are within-category consistency/diversity descriptors, not accuracy.",
        "",
        "| Category | N | Unique | Unique rate | Mode rate | Pairwise exact | Entropy |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for cohort in EXPECTED_COHORTS:
        row = topology["per_category"][cohort]
        lines.append(
            f"| {cohort} | {row['sample_count']} | {row['unique_signature_count']} | "
            f"{fmt(row['unique_signature_rate'])} | {fmt(row['mode_rate'])} | "
            f"{fmt(row['pairwise_exact_rate'])} | {fmt(row['normalized_entropy'])} |"
        )
    lines.extend(
        [
            "",
            f"- Equal-category macro unique-signature rate: {fmt(macro['unique_signature_rate_macro'])}",
            f"- Equal-category macro mode rate: {fmt(macro['mode_rate_macro'])}",
            f"- Equal-category macro pairwise exact rate: {fmt(macro['pairwise_exact_rate_macro'])}",
            f"- Equal-category macro normalized entropy: {fmt(macro['normalized_entropy_macro'])}",
            "",
            "## Independent-gold feasibility",
            "",
            f"- Independent structured gold found: `{str(gold['independent_gold_found']).lower()}`",
            f"- Targeted official release/repository search matches: {gold['repository_search_match_count']}",
            f"- Parent-Child Edge F1: {gold['metric_status']['parent_child_edge_f1']}",
            f"- Hierarchy Exact Match: {gold['metric_status']['hierarchy_exact_match']}",
            f"- Semantic Nesting Accuracy: {gold['metric_status']['semantic_nesting_accuracy']}",
            "",
            "The generated model source, materialized URDF, and compile report are on the prediction path and are excluded as gold. Free-form prompts and scalar ratings are not complete structured annotations.",
            "",
        ]
    )
    return "\n".join(lines)


def self_test(topology_consistency: Any) -> None:
    rows = [
        {"canonical_topology_signature": "A"},
        {"canonical_topology_signature": "A"},
        {"canonical_topology_signature": "B"},
    ]
    result = topology_consistency(rows)
    assert result["sample_count"] == 3
    assert result["unique_signature_count"] == 2
    assert abs(result["mode_rate"] - 2 / 3) < 1e-12
    assert abs(result["pairwise_exact_rate"] - 1 / 3) < 1e-12


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    analyze_urdf, aggregate, topology_consistency, evaluator_hash = load_shared_evaluator()
    self_test(topology_consistency)
    if args.self_test:
        print(f"self-test: ok; shared_evaluator_sha256={evaluator_hash}")
        return 0

    input_dir = contained(args.input)
    output_dir = contained(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    selection_path = contained(input_dir / "selection_manifest.jsonl")
    source_records_path = contained(input_dir / "records.jsonl")
    source_summary_path = contained(input_dir / "summary.json")
    selections = load_jsonl(selection_path)
    source_records = load_jsonl(source_records_path)
    source_summary = load_json(source_summary_path)
    selection_hash = sha256(selection_path)
    if selection_hash != FROZEN_SELECTION_SHA256:
        raise ValueError(f"frozen selection hash mismatch: {selection_hash}")
    if len(selections) != 30 or len(source_records) != 30:
        raise ValueError("expected exactly 30 frozen selections and source records")
    if Counter(str(row["cohort"]) for row in selections) != Counter(EXPECTED_COHORTS):
        raise ValueError("frozen per-category counts changed")
    selection_ids = [str(row["record_id"]) for row in selections]
    if len(set(selection_ids)) != 30:
        raise ValueError("duplicate record IDs in frozen selection")
    source_by_id = {str(row["record_id"]): row for row in source_records}
    if set(source_by_id) != set(selection_ids):
        raise ValueError("source records do not match frozen selection")
    if source_summary["selected_cohort"]["selection_manifest_sha256"] != selection_hash:
        raise ValueError("source summary selection hash mismatch")

    extended_records: list[dict[str, Any]] = []
    for selection in selections:
        record_id = str(selection["record_id"])
        source = source_by_id[record_id]
        urdf = contained(Path(str(source["model_urdf"])))
        current_hash = sha256(urdf)
        if current_hash != source["model_urdf_sha256"]:
            raise ValueError(f"URDF hash mismatch: {record_id}")
        extended_records.append(
            {
                "cohort": selection["cohort"],
                "category_slug": selection["category_slug"],
                "record_id": record_id,
                "rank": selection["rank"],
                "rank_sha256": selection["rank_sha256"],
                "rating": selection["rating"],
                "model_urdf": str(urdf),
                "model_urdf_sha256": current_hash,
                **analyze_urdf(urdf),
            }
        )

    per_category_rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in extended_records:
        per_category_rows[str(row["cohort"])].append(row)
    overall = aggregate(extended_records, requested_count=30)
    per_category = {
        cohort: aggregate(per_category_rows[cohort], requested_count=EXPECTED_COHORTS[cohort])
        for cohort in EXPECTED_COHORTS
    }
    per_category_topology = {
        cohort: topology_consistency(per_category_rows[cohort])
        for cohort in EXPECTED_COHORTS
    }
    gold = independent_gold_audit(input_dir)
    runner_path = contained(Path(__file__))
    summary = {
        "status": "COMPLETE",
        "protocol": PROTOCOL,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "claim_boundary": (
            "descriptive topology and within-category consistency on frozen official-release outputs; "
            "not gold-referenced hierarchy correctness and not a common-prompt rerun"
        ),
        "evaluation_unit": "one frozen, officially released Articraft record and its verified materialized URDF",
        "frozen_cohort": {
            "record_count": 30,
            "per_category_n": 6,
            "category_counts": EXPECTED_COHORTS,
            "selection_manifest_sha256": selection_hash,
            "selection_rule": source_summary["selected_cohort"]["selection_rule"],
            "selection_unchanged": True,
            "urdf_hash_verified_count": 30,
        },
        "overall": overall,
        "overall_valid_tree_ranges": metric_ranges(extended_records),
        "per_category": per_category,
        "topology_consistency": {
            "definition": (
                "shared evaluator name-free rooted signature retaining exact joint types and "
                "visual/group node roles; within-category only"
            ),
            "per_category": per_category_topology,
            "category_macro": category_macro_topology(per_category_topology),
            "pooled_all_categories": topology_consistency(extended_records),
        },
        "independent_gold_audit": gold,
        "provenance": {
            "source_summary": str(source_summary_path),
            "source_summary_sha256": sha256(source_summary_path),
            "source_records": str(source_records_path),
            "source_records_sha256": sha256(source_records_path),
            "selection_manifest": str(selection_path),
            "runner": str(runner_path),
            "runner_sha256": sha256(runner_path),
            "shared_extended_evaluator": str(SHARED_EVALUATOR),
            "shared_extended_evaluator_sha256": evaluator_hash,
            "dataset_commit": source_summary["provenance"]["dataset_commit"],
            "paper_harness_commit": source_summary["provenance"]["paper_harness_commit"],
        },
    }
    records_output = contained(output_dir / "extended_records.jsonl")
    summary_output = contained(output_dir / "extended_summary.json")
    report_output = contained(output_dir / "extended_report.md")
    gold_output = contained(output_dir / "extended_gold_audit.json")
    records_output.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in extended_records),
        encoding="utf-8",
    )
    summary_output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report_output.write_text(build_report(summary), encoding="utf-8")
    gold_output.write_text(json.dumps(gold, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
