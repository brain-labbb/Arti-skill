#!/usr/bin/env python3
"""Combine and bootstrap four frozen PartNet semantic-alignment panels."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import random
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/combined"
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260811
CATEGORIES = [
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
]
METHOD_INPUTS = {
    "PV-A": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva/partnet_scores",
    "LAM": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/lam/partnet_scores",
    "Articraft": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/articraft/partnet_scores",
    "Infinite Mobility": (
        EXP_ROOT
        / "runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel/partnet_scores"
    ),
}


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def mean(values: list[float]) -> float:
    return sum(values) / len(values)


def quantile(sorted_values: list[float], probability: float) -> float:
    position = (len(sorted_values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return sorted_values[lower] * (1 - weight) + sorted_values[upper] * weight


def requested_value(row: dict[str, Any], metric: str) -> float:
    if metric == "role_coverage":
        return float(row.get("semantic_role_coverage") or 0.0)
    if metric == "scorable":
        return float(bool(row.get("scorable")))
    if metric == "edge_f1":
        return float(row.get("parent_child_edge_f1") or 0.0)
    if metric == "coverage_weighted_induced_edge_f1":
        return float(row.get("parent_child_edge_f1") or 0.0) * float(
            row.get("semantic_role_coverage") or 0.0
        )
    if metric == "exact":
        return float(bool(row.get("hierarchy_exact_match")))
    if metric == "nesting":
        return float(row.get("semantic_nesting_accuracy") or 0.0)
    raise KeyError(metric)


def validate_records(method: str, records: list[dict[str, Any]]) -> None:
    if len(records) != 30:
        raise ValueError(f"{method}: expected 30 records, found {len(records)}")
    counts = {
        category: sum(row["category"] == category for row in records)
        for category in CATEGORIES
    }
    if set(counts.values()) != {6} or {row["category"] for row in records} != set(CATEGORIES):
        raise ValueError(f"{method}: invalid category counts {counts}")
    if {row["method"] for row in records} != {method}:
        raise ValueError(f"{method}: method label mismatch")


def stratified_resample(
    records: list[dict[str, Any]], rng: random.Random
) -> list[dict[str, Any]]:
    sampled = []
    for category in CATEGORIES:
        population = [row for row in records if row["category"] == category]
        sampled.extend(rng.choice(population) for _ in range(6))
    return sampled


def bootstrap_metrics(
    records_by_method: dict[str, list[dict[str, Any]]]
) -> tuple[dict[str, Any], dict[str, Any]]:
    metric_names = [
        "role_coverage",
        "scorable",
        "edge_f1",
        "coverage_weighted_induced_edge_f1",
        "exact",
        "nesting",
    ]
    estimates = {
        method: {
            metric: mean([requested_value(row, metric) for row in records])
            for metric in metric_names
        }
        for method, records in records_by_method.items()
    }
    samples = {
        method: {metric: [] for metric in metric_names}
        for method in records_by_method
    }
    rng = random.Random(BOOTSTRAP_SEED)
    for _ in range(BOOTSTRAP_REPLICATES):
        for method, records in records_by_method.items():
            resampled = stratified_resample(records, rng)
            for metric in metric_names:
                samples[method][metric].append(
                    mean([requested_value(row, metric) for row in resampled])
                )

    intervals = {}
    for method in records_by_method:
        intervals[method] = {}
        for metric in metric_names:
            values = sorted(samples[method][metric])
            intervals[method][metric] = {
                "estimate": estimates[method][metric],
                "ci95_percentile": [quantile(values, 0.025), quantile(values, 0.975)],
            }

    differences = {}
    pva_samples = samples["PV-A"]["coverage_weighted_induced_edge_f1"]
    for baseline in ("LAM", "Articraft", "Infinite Mobility"):
        values = sorted(
            pva - other
            for pva, other in zip(
                pva_samples, samples[baseline]["coverage_weighted_induced_edge_f1"]
            )
        )
        differences[f"PV-A_minus_{baseline}"] = {
            "estimate": (
                estimates["PV-A"]["coverage_weighted_induced_edge_f1"]
                - estimates[baseline]["coverage_weighted_induced_edge_f1"]
            ),
            "ci95_percentile": [quantile(values, 0.025), quantile(values, 0.975)],
            "interpretation": "independent category-stratified bootstrap difference",
        }
    return intervals, differences


def fmt(value: float | None) -> str:
    return "N/A" if value is None else f"{100 * value:.1f}"


def fmt_ci(entry: dict[str, Any]) -> str:
    low, high = entry["ci95_percentile"]
    return f"{fmt(entry['estimate'])} [{fmt(low)}, {fmt(high)}]"


def main() -> int:
    records_by_method = {}
    input_evidence = {}
    summaries = {}
    for method, directory in METHOD_INPUTS.items():
        score_dir = contained(directory)
        records_path = contained(score_dir / "records.jsonl")
        summary_path = contained(score_dir / "summary.json")
        records = load_jsonl(records_path)
        validate_records(method, records)
        records_by_method[method] = records
        summaries[method] = json.loads(summary_path.read_text(encoding="utf-8"))
        input_evidence[method] = {
            "records_path": str(records_path),
            "records_sha256": sha256(records_path),
            "summary_path": str(summary_path),
            "summary_sha256": sha256(summary_path),
        }

    protocol_hashes = {row["partnet_protocol_sha256"] for row in summaries.values()}
    scorer_hashes = {row["scorer_sha256"] for row in summaries.values()}
    if len(protocol_hashes) != 1 or len(scorer_hashes) != 1:
        raise ValueError("methods were not scored with identical protocol/scorer hashes")

    intervals, differences = bootstrap_metrics(records_by_method)
    payload = {
        "protocol_id": "nano3d_partnet_hierarchy_correctness_combined_v1",
        "claim_boundary": next(iter(summaries.values()))["claim_boundary"],
        "categories": CATEGORIES,
        "requested_per_category": 6,
        "requested_per_method": 30,
        "bootstrap": {
            "design": "independent category-stratified asset bootstrap with replacement",
            "replicates": BOOTSTRAP_REPLICATES,
            "seed": BOOTSTRAP_SEED,
            "interval": "95% percentile",
            "category_weighting": "five categories equally weighted; six draws per category",
            "cross_method_pairing": "none; each method is independently resampled within category",
            "pairwise_interval_multiplicity": "unadjusted exploratory intervals; no multiple-comparison correction",
        },
        "partnet_protocol_sha256": next(iter(protocol_hashes)),
        "scorer_sha256": next(iter(scorer_hashes)),
        "methods": {
            method: {
                "overall": summaries[method]["overall"],
                "bootstrap_intervals": intervals[method],
                "selection_manifest_sha256": summaries[method][
                    "selection_manifest_sha256"
                ],
                "prediction_side_link_labels_sha256": summaries[method].get(
                    "prediction_side_link_labels_sha256"
                ),
                "prediction_side_link_label_count": summaries[method].get(
                    "prediction_side_link_label_count", 0
                ),
            }
            for method in METHOD_INPUTS
        },
        "pairwise_primary_metric_differences": differences,
        "input_evidence": input_evidence,
    }

    output = contained(OUTPUT, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "summary.json"
    summary_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    lines = [
        "# PartNet-ontology-referenced semantic hierarchy alignment panel",
        "",
        "Five categories, six frozen assets per category and method (N=30). "
        "All requested assets remain in the denominator; no failure replacement.",
        "",
        "| Method | Available | Role cov. % | Scorable % | Coverage-Weighted Induced Edge F1 % (95% CI) | Semantic-Parent Alignment % requested |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for method in METHOD_INPUTS:
        overall = summaries[method]["overall"]
        boot = intervals[method]
        lines.append(
            f"| {method} | {overall['available_count']}/30 | "
            f"{fmt(boot['role_coverage']['estimate'])} | "
            f"{fmt(boot['scorable']['estimate'])} | "
            f"{fmt_ci(boot['coverage_weighted_induced_edge_f1'])} | "
            f"{fmt(boot['nesting']['estimate'])} |"
        )
    lines.extend(
        [
            "",
            "| Method | Induced Edge F1 % requested | Induced Edge F1 % conditional | Induced Exact % requested |",
            "|---|---:|---:|---:|",
        ]
    )
    for method in METHOD_INPUTS:
        overall = summaries[method]["overall"]
        boot = intervals[method]
        lines.append(
            f"| {method} | {fmt(boot['edge_f1']['estimate'])} | "
            f"{fmt(overall['parent_child_edge_f1_conditional_macro'])} | "
            f"{fmt(boot['exact']['estimate'])} |"
        )
    lines.extend(
        [
            "",
            "Coverage-Weighted Induced Edge F1 is the per-asset induced-edge F1 multiplied "
            "by mapped-link coverage, then macro-averaged over all 30 requested assets. "
            "Unavailable and unscorable assets contribute zero.",
            "",
            "The reference is a category-level PartNet ontology, not instance-matched "
            "PartNet geometry or human annotation. Exact and nesting results therefore "
            "measure alignment of mapped predicted roles with the induced semantic ontology, "
            "not instance-level kinematic hierarchy correctness.",
            "",
            "Infinite Mobility uses only package-local raw part names to decode opaque "
            "URDF link IDs; the mapping is prediction-side metadata, hash-pinned, and is "
            "never used as gold.",
            "",
            "Bootstrap resampling is independent across methods: each replicate draws six "
            "assets with replacement within each of five equally weighted categories. "
            "Pairwise percentile intervals are exploratory and unadjusted for multiplicity.",
        ]
    )
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "methods": list(METHOD_INPUTS)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
