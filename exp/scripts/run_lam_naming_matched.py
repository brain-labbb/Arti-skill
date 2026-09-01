#!/usr/bin/env python3
"""Build the frozen five-category LAM Naming matched cohort."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import random
import statistics
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table2_naming_matched_protocol_v1.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/lam_naming_matched_v1"
RELEASE_SUMMARY = REPO_ROOT / "exp/runtime/lam_naming_v1/summary.json"
RUN_PIPELINE = REPO_ROOT / ".cache/table6_sources/lam/code/run_pipeline.py"
SEMANTIC_FIELDS = (
    "semantic_precision",
    "semantic_recall",
    "naming_richness",
    "functional_core_coverage",
    "instance_discriminability",
    "over_segmentation_rate",
)


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def require_file(path: Path) -> Path:
    resolved = contained(path)
    if resolved.is_symlink() or not resolved.is_file():
        raise RuntimeError(f"required regular file missing: {resolved}")
    return resolved


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(require_file(path).read_bytes())


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(require_file(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"expected JSON object: {path}")
    return value


def write_atomic(path: Path, text: str) -> None:
    resolved = contained(path)
    resolved.parent.mkdir(parents=True, exist_ok=True)
    temporary = resolved.with_name(f".{resolved.name}.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(resolved)


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def ratio(numerator: int, denominator: int) -> float:
    if denominator <= 0:
        raise RuntimeError("zero denominator in Naming metric")
    return numerator / denominator


def source_path(relative: str) -> Path:
    return contained(REPO_ROOT / relative)


def assert_protocol_sources(protocol: dict[str, Any]) -> dict[str, str]:
    observed: dict[str, str] = {}
    base = protocol["base_naming_protocol"]
    base_path = source_path(base["path"])
    observed[base["path"]] = sha256_file(base_path)
    if observed[base["path"]] != base["sha256"]:
        raise RuntimeError("base Naming protocol hash mismatch")

    for evidence in protocol["source_evidence"].values():
        path = source_path(evidence["path"])
        observed[evidence["path"]] = sha256_file(path)
        if observed[evidence["path"]] != evidence["sha256"]:
            raise RuntimeError(f"source hash mismatch: {evidence['path']}")

    for category in protocol["design"]["canonical_categories"]:
        ours = protocol["taxonomy"][category]["ours"]
        path = source_path(ours["template"])
        observed[ours["template"]] = sha256_file(path)
        if observed[ours["template"]] != ours["sha256"]:
            raise RuntimeError(f"template hash mismatch: {ours['template']}")
    return dict(sorted(observed.items()))


def read_release_records(path: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in require_file(path).read_text(encoding="utf-8").splitlines()]
    if not rows or not all(isinstance(row, dict) for row in rows):
        raise RuntimeError("empty or malformed LAM release records")
    ids = [row["object_release_id"] for row in rows]
    if len(ids) != len(set(ids)):
        raise RuntimeError("duplicate LAM object_release_id")
    return rows


def viable_manifest_counts(path: Path) -> tuple[dict[str, int], set[str]]:
    counts: Counter[str] = Counter()
    ids: set[str] = set()
    with require_file(path).open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row["tier"] == "viable":
                counts[row["category"]] += 1
                ids.add(row["object_release_id"])
    return dict(counts), ids


def selection_digest(category: str, row: dict[str, Any]) -> str:
    value = (
        "nano3d-table2-naming-matched-v1|LAM|"
        f"{category}|{row['object_release_id']}|{row['urdf_sha256']}"
    )
    return sha256_bytes(value.encode("utf-8"))


def select_records(
    protocol: dict[str, Any], rows: list[dict[str, Any]], manifest_ids: set[str]
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    categories = protocol["design"]["canonical_categories"]
    k = protocol["design"]["assets_per_category_per_method"]
    category_for_slug: dict[str, str] = {}
    for category in categories:
        for slug in protocol["taxonomy"][category]["lam_whitelist"]:
            if slug in category_for_slug:
                raise RuntimeError(f"LAM whitelist overlap: {slug}")
            category_for_slug[slug] = category

    pools: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if row["object_release_id"] not in manifest_ids:
            raise RuntimeError(f"release record absent from viable manifest: {row['object_release_id']}")
        category = category_for_slug.get(row["category"])
        if category is not None:
            pools[category].append(row)

    selected: list[dict[str, Any]] = []
    audit: dict[str, Any] = {}
    for category in categories:
        ranked = sorted(
            pools[category],
            key=lambda row: (
                selection_digest(category, row),
                row["object_release_id"],
                row["urdf_sha256"],
            ),
        )
        if len(ranked) < k:
            raise RuntimeError(f"LAM pool below K for {category}: {len(ranked)} < {k}")
        chosen = ranked[:k]
        for rank, row in enumerate(chosen, start=1):
            output = dict(row)
            output["canonical_category"] = category
            output["selection_rank"] = rank
            output["selection_digest"] = selection_digest(category, row)
            selected.append(output)
        audit[category] = {
            "whitelist": protocol["taxonomy"][category]["lam_whitelist"],
            "eligible_pool_count": len(ranked),
            "selected_count": len(chosen),
            "selected_release_ids": [row["object_release_id"] for row in chosen],
            "first_excluded_digest": selection_digest(category, ranked[k]) if len(ranked) > k else None,
        }
    return selected, audit


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    parts = [int(row["renderable_part_count"]) for row in rows]
    named = [int(row["named_renderable_part_count"]) for row in rows]
    placeholders = [int(row["placeholder_renderable_part_count"]) for row in rows]
    total_parts = sum(parts)
    total_named = sum(named)
    return {
        "assets": len(rows),
        "total_renderable_parts": total_parts,
        "total_named_renderable_parts": total_named,
        "parts_per_asset_mean": statistics.fmean(parts),
        "parts_per_asset_median": statistics.median(parts),
        "nameability_micro": ratio(total_named, total_parts),
        "nameability_asset_macro": statistics.fmean(float(row["nameability"]) for row in rows),
        "placeholder_renderable_parts": sum(placeholders),
        "assets_with_placeholder_parts": sum(value > 0 for value in placeholders),
        "fully_nameable_assets": sum(value == 0 for value in placeholders),
        "parts_per_asset_min": min(parts),
        "parts_per_asset_max": max(parts),
    }


def category_bootstrap(
    selected: list[dict[str, Any]], protocol: dict[str, Any]
) -> dict[str, Any]:
    config = protocol["bootstrap"]
    categories = protocol["design"]["canonical_categories"]
    grouped = {category: [r for r in selected if r["canonical_category"] == category] for category in categories}
    rng = random.Random(config["seed"])
    parts_means: list[float] = []
    micro_scores: list[float] = []
    category_macro_scores: list[float] = []
    for _ in range(config["resamples"]):
        sampled_categories = [rng.choice(categories) for _ in categories]
        sampled = [row for category in sampled_categories for row in grouped[category]]
        total_parts = sum(row["renderable_part_count"] for row in sampled)
        total_named = sum(row["named_renderable_part_count"] for row in sampled)
        parts_means.append(total_parts / len(sampled))
        micro_scores.append(ratio(total_named, total_parts))
        category_macro_scores.append(
            statistics.fmean(
                ratio(
                    sum(row["named_renderable_part_count"] for row in grouped[category]),
                    sum(row["renderable_part_count"] for row in grouped[category]),
                )
                for category in sampled_categories
            )
        )
    alpha = (1.0 - config["confidence"]) / 2.0
    return {
        "method": "canonical-category cluster bootstrap with replacement",
        "cluster_count": len(categories),
        "assets_per_cluster": protocol["design"]["assets_per_category_per_method"],
        "resamples": config["resamples"],
        "confidence": config["confidence"],
        "seed": config["seed"],
        "parts_per_asset_mean": [percentile(parts_means, alpha), percentile(parts_means, 1.0 - alpha)],
        "nameability_micro": [percentile(micro_scores, alpha), percentile(micro_scores, 1.0 - alpha)],
        "nameability_category_macro": [
            percentile(category_macro_scores, alpha),
            percentile(category_macro_scores, 1.0 - alpha),
        ],
    }


def generation_preflight() -> dict[str, Any]:
    source = require_file(RUN_PIPELINE).read_text(encoding="utf-8")
    config_path = contained(RUN_PIPELINE.parent / "config.yaml")
    evidence = {
        "config_manager_import": "from utils.config_manager import ConfigManager" in source,
        "unconditional_config_yaml_initialization": 'ConfigManager("config.yaml")' in source,
        "agent_initialization_present": "LinkerGeneratorAgent(self.config)" in source,
        "config_yaml_present": config_path.exists(),
    }
    runnable = all(
        [
            evidence["config_manager_import"],
            evidence["unconditional_config_yaml_initialization"],
            evidence["agent_initialization_present"],
            evidence["config_yaml_present"],
        ]
    )
    return {
        "status": "NOT_RUN" if not runnable else "AVAILABLE_NOT_RUN",
        "new_prompt_generation_without_api_configuration": "BLOCKED_NO_API_CONFIG" if not runnable else "NOT_TESTED",
        "reason": "official entry point unconditionally loads config.yaml before initializing provider-backed agents; config.yaml is absent" if not runnable else "configuration exists, but generation is outside this release-resampling evaluation",
        "config_contents_read": False,
        "environment_credentials_inspected": False,
        "network_accessed": False,
        "evidence": evidence,
        "run_pipeline_path": str(RUN_PIPELINE.relative_to(REPO_ROOT)),
        "run_pipeline_sha256": sha256_file(RUN_PIPELINE),
    }


def report_text(summary: dict[str, Any]) -> str:
    direct = summary["direct_metrics"]
    lines = [
        "# LAM Naming matched cohort v1",
        "",
        f"Status: **{summary['status']}**",
        "",
        "This is a category-matched deterministic release resample, not a same-prompt or same-seed comparison.",
        "",
        "## Coverage",
        "",
        f"- Canonical categories: {summary['coverage']['canonical_categories']}",
        f"- Selected assets: {summary['coverage']['selected_assets']} / {summary['coverage']['expected_assets']}",
        f"- Assets per category: {summary['coverage']['assets_per_category']}",
        "",
        "## Direct metrics",
        "",
        f"- Renderable URDF links: {direct['total_renderable_parts']}",
        f"- Parts per asset: mean {direct['parts_per_asset_mean']:.6f}, median {direct['parts_per_asset_median']}",
        f"- Nameability: micro {direct['nameability_micro']:.6f}, asset macro {direct['nameability_asset_macro']:.6f}",
        f"- Placeholder links: {direct['placeholder_renderable_parts']} across {direct['assets_with_placeholder_parts']} assets",
        "",
        "## Category breakdown",
        "",
        "| category | n | parts mean | nameability micro | placeholders |",
        "|---|---:|---:|---:|---:|",
    ]
    for category, row in summary["per_category"].items():
        lines.append(
            f"| {category} | {row['assets']} | {row['parts_per_asset_mean']:.6f} | "
            f"{row['nameability_micro']:.6f} | {row['placeholder_renderable_parts']} |"
        )
    lines.extend(
        [
            "",
            "## Evidence limits",
            "",
            "- Cross-seed consistency: N/A; release records expose no frozen seed identity.",
            "- Semantic metrics: N/A; no LAM-linked independent gold and three complete blind judges.",
            f"- New prompt generation: {summary['generation_preflight']['new_prompt_generation_without_api_configuration']} (not run).",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    protocol_path = require_file(args.protocol)
    output = contained(args.output)
    protocol = read_json(protocol_path)
    if protocol.get("protocol_id") != "nano3d_table2_naming_matched_v1":
        raise RuntimeError("unexpected matched protocol id")
    observed_hashes = assert_protocol_sources(protocol)

    release_summary = read_json(RELEASE_SUMMARY)
    if release_summary.get("status") != "COMPLETE":
        raise RuntimeError("LAM full release Naming evaluation is not COMPLETE")
    release_records_path = source_path(protocol["source_evidence"]["lam_full_naming_records"]["path"])
    release_records = read_release_records(release_records_path)
    manifest_path = source_path(protocol["source_evidence"]["lam_manifest_csv"]["path"])
    manifest_counts, viable_ids = viable_manifest_counts(manifest_path)
    if len(release_records) != release_summary["coverage"]["requested_viable_assets"]:
        raise RuntimeError("full release record count disagrees with release summary")

    selected, pool_audit = select_records(protocol, release_records, viable_ids)
    expected = protocol["design"]["assets_per_method"]
    k = protocol["design"]["assets_per_category_per_method"]
    categories = protocol["design"]["canonical_categories"]
    if len(selected) != expected or any(sum(r["canonical_category"] == c for r in selected) != k for c in categories):
        raise RuntimeError("matched cohort cardinality gate failed")
    if not all(r["parse_success"] and r["naming_evaluable"] for r in selected):
        raise RuntimeError("selected LAM record is not Naming-evaluable")

    direct = aggregate(selected)
    per_category = {category: aggregate([r for r in selected if r["canonical_category"] == category]) for category in categories}
    category_macro = statistics.fmean(row["nameability_micro"] for row in per_category.values())
    direct["nameability_category_macro"] = category_macro
    bootstrap = category_bootstrap(selected, protocol)
    generation = generation_preflight()

    records_text = "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in selected)
    cohort = {
        "protocol_id": protocol["protocol_id"],
        "method": "LAM",
        "comparison_label": protocol["comparison_label"],
        "selection_policy": protocol["selection"]["lam"],
        "pool_audit": pool_audit,
        "manifest_viable_category_counts": {
            slug: manifest_counts.get(slug, 0)
            for category in categories
            for slug in protocol["taxonomy"][category]["lam_whitelist"]
        },
        "selected": [
            {
                "canonical_category": row["canonical_category"],
                "selection_rank": row["selection_rank"],
                "selection_digest": row["selection_digest"],
                "object_release_id": row["object_release_id"],
                "source_category": row["category"],
                "urdf_sha256": row["urdf_sha256"],
            }
            for row in selected
        ],
    }
    stable = {
        "coverage": {"selected_assets": len(selected), "expected_assets": expected},
        "direct_metrics": direct,
        "per_category": per_category,
        "bootstrap_ci": bootstrap,
        "selected": cohort["selected"],
    }
    summary = {
        "protocol_id": "nano3d_lam_naming_matched_v1",
        "shared_protocol_id": protocol["protocol_id"],
        "status": "COMPLETE",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "comparison_scope": "five-category equal-N deterministic official-release resample; not same-prompt and not same-seed",
        "network_accessed": False,
        "generated_code_executed": False,
        "paper_values_reused": False,
        "coverage": {
            "canonical_categories": len(categories),
            "category_names": categories,
            "assets_per_category": k,
            "selected_assets": len(selected),
            "expected_assets": expected,
            "full_release_records_checked": len(release_records),
            "all_selected_parse_success": True,
            "all_selected_naming_evaluable": True,
        },
        "direct_metrics": direct,
        "per_category": per_category,
        "bootstrap_ci": bootstrap,
        "cross_seed": {
            "eligible": False,
            "status": "N/A",
            "reason": "LAM release records expose no frozen reusable template/factory seed identity",
        },
        "semantic_metrics": {field: None for field in SEMANTIC_FIELDS},
        "semantic_status": "N/A: no LAM-linked output-independent role gold and no three complete independent blind judges",
        "generation_preflight": generation,
        "provenance": {
            "shared_protocol_sha256": sha256_file(protocol_path),
            "source_hashes": observed_hashes,
            "release_summary_sha256": sha256_file(RELEASE_SUMMARY),
            "runner_sha256": sha256_file(Path(__file__)),
            "records_sha256": sha256_bytes(records_text.encode("utf-8")),
            "cohort_manifest_sha256": None,
            "stable_results_sha256": sha256_bytes(json.dumps(stable, sort_keys=True, separators=(",", ":")).encode("utf-8")),
        },
    }

    cohort_text = json.dumps(cohort, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    summary["provenance"]["cohort_manifest_sha256"] = sha256_bytes(cohort_text.encode("utf-8"))
    report = report_text(summary)
    write_atomic(output / "records.jsonl", records_text)
    write_atomic(output / "cohort_manifest.json", cohort_text)
    write_atomic(output / "summary.json", json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n")
    write_atomic(output / "report.md", report)
    print(json.dumps({"status": summary["status"], "assets": len(selected), "output": str(output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
