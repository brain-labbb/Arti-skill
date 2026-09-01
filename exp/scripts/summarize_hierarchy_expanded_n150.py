#!/usr/bin/env python3
"""Aggregate the balanced expanded-N hierarchy experiment without rank pairing.

PV-A, Articraft, and Infinite Mobility form the only generated-method ranking
panel. LAM's unbalanced official-release audit and Artiverse's curated-data
reference are copied into explicitly separate supplementary panels.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
from typing import Any, Callable


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
RUNTIME_ROOT = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150"
OUTPUT = RUNTIME_ROOT / "aggregate"
PAPER_MARKDOWN = EXP_ROOT / "Table3_Hierarchy_Expanded_N.md"
CATEGORIES = [
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
]
METHODS = ("PV-A", "Articraft", "Infinite Mobility")
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260813
METRICS = (
    "available_requested",
    "valid_requested",
    "role_coverage_requested",
    "scorable_requested",
    "coverage_weighted_induced_edge_f1_requested",
    "induced_edge_f1_requested",
    "induced_exact_requested",
    "semantic_parent_alignment_requested",
)
STRUCTURE_BOOTSTRAP_METRICS = (
    "valid_available",
    "node_count_mean_valid",
    "semantic_depth_mean_valid",
    "movable_joint_count_mean_valid",
)
METHOD_PATHS = {
    "PV-A": {
        "root": RUNTIME_ROOT / "pva",
        "structure": "extended_structure_records.jsonl",
        "alignment": "partnet_scores/records.jsonl",
    },
    "Articraft": {
        "root": RUNTIME_ROOT / "articraft",
        "structure": "extended_structure_records.jsonl",
        "alignment": "partnet_scores/records.jsonl",
    },
    "Infinite Mobility": {
        "root": RUNTIME_ROOT / "infinite_mobility",
        "structure": "structure_records.jsonl",
        "alignment": "partnet_alignment_records.jsonl",
    },
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


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not all(isinstance(row, dict) for row in rows):
        raise ValueError(f"{path}: expected one JSON object per line")
    return rows


def verification_document_passed(document: dict[str, Any]) -> bool:
    signals: list[bool] = []
    if "status" in document:
        signals.append(document["status"] == "PASS")
    if "passed" in document:
        signals.append(document["passed"] is True)
    if "all_checks_pass" in document:
        signals.append(document["all_checks_pass"] is True)

    checks = document.get("checks")
    if isinstance(checks, dict) and checks:
        boolean_checks = [value for value in checks.values() if isinstance(value, bool)]
        if boolean_checks:
            signals.append(len(boolean_checks) == len(checks) and all(boolean_checks))
    elif isinstance(checks, list):
        explicit_checks = [
            item["passed"]
            for item in checks
            if isinstance(item, dict) and isinstance(item.get("passed"), bool)
        ]
        if explicit_checks:
            signals.append(all(explicit_checks))

    for key in ("main_verification", "replay_verification"):
        nested = document.get(key)
        if isinstance(nested, dict):
            signals.append(verification_document_passed(nested))
    return bool(signals) and all(signals)


def unique_index(
    rows: list[dict[str, Any]], *, method: str, source: str
) -> dict[str, dict[str, Any]]:
    index = {str(row["sample_id"]): row for row in rows}
    if len(index) != len(rows):
        raise ValueError(f"{method}: duplicate sample_id values in {source}")
    return index


def mean(values: list[float]) -> float:
    if not values:
        raise ValueError("cannot average an empty sequence")
    return sum(values) / len(values)


def quantile(sorted_values: list[float], probability: float) -> float:
    position = (len(sorted_values) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def requested_value(row: dict[str, Any], metric: str) -> float:
    if metric == "available_requested":
        return float(bool(row.get("available")))
    if metric == "valid_requested":
        return float(bool(row.get("valid_tree")))
    if metric == "role_coverage_requested":
        return float(row.get("semantic_role_coverage") or 0.0)
    if metric == "scorable_requested":
        return float(bool(row.get("scorable")))
    if metric == "coverage_weighted_induced_edge_f1_requested":
        return float(row.get("induced_parent_child_edge_f1") or 0.0) * float(
            row.get("semantic_role_coverage") or 0.0
        )
    if metric == "induced_edge_f1_requested":
        return float(row.get("induced_parent_child_edge_f1") or 0.0)
    if metric == "induced_exact_requested":
        return float(bool(row.get("induced_hierarchy_exact_match")))
    if metric == "semantic_parent_alignment_requested":
        return float(row.get("semantic_nesting_accuracy") or 0.0)
    raise KeyError(metric)


def category_macro_valid_mean(
    rows: list[dict[str, Any]], field: str
) -> float | None:
    category_means: list[float] = []
    for category in CATEGORIES:
        values = [
            float(row[field])
            for row in rows
            if row["category"] == category
            and row.get("valid_tree")
            and row.get(field) is not None
        ]
        if values:
            category_means.append(mean(values))
    return mean(category_means) if category_means else None


def compute_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [row for row in rows if row.get("valid_tree")]
    available_count = sum(bool(row.get("available")) for row in rows)
    result: dict[str, Any] = {
        metric: mean([requested_value(row, metric) for row in rows])
        for metric in METRICS
    }
    result.update(
        {
            "requested_count": len(rows),
            "available_count": sum(bool(row.get("available")) for row in rows),
            "valid_tree_count": len(valid),
            "scorable_count": sum(bool(row.get("scorable")) for row in rows),
            "valid_available": (
                len(valid) / available_count if available_count else None
            ),
            "node_count_mean_valid": category_macro_valid_mean(rows, "node_count"),
            "semantic_depth_mean_valid": category_macro_valid_mean(
                rows, "semantic_depth"
            ),
            "movable_joint_count_mean_valid": category_macro_valid_mean(
                rows, "movable_edge_count"
            ),
            "node_count_mean_pooled_valid": (
                mean([float(row["node_count"]) for row in valid]) if valid else None
            ),
            "semantic_depth_mean_pooled_valid": (
                mean([float(row["semantic_depth"]) for row in valid]) if valid else None
            ),
            "movable_joint_count_mean_pooled_valid": (
                mean([float(row["movable_edge_count"]) for row in valid])
                if valid
                else None
            ),
            "structure_primary_aggregation": (
                "equal-category macro of within-category valid-tree means"
            ),
            "structure_pooled_fields_role": "diagnostic only",
        }
    )
    return result


def stratified_resample(
    rows: list[dict[str, Any]], rng: random.Random
) -> list[dict[str, Any]]:
    sampled: list[dict[str, Any]] = []
    for category in CATEGORIES:
        population = [row for row in rows if row["category"] == category]
        if len(population) != 30:
            raise ValueError(f"{category}: expected 30 records, found {len(population)}")
        sampled.extend(rng.choice(population) for _ in range(30))
    return sampled


def bootstrap_methods(
    records_by_method: dict[str, list[dict[str, Any]]],
    *,
    replicates: int = BOOTSTRAP_REPLICATES,
    seed: int = BOOTSTRAP_SEED,
    resample_audit: list[tuple[str, tuple[str, ...]]] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    estimates = {
        method: compute_metrics(rows) for method, rows in records_by_method.items()
    }
    bootstrap_metrics = METRICS + STRUCTURE_BOOTSTRAP_METRICS
    samples = {
        method: {metric: [] for metric in bootstrap_metrics}
        for method in records_by_method
    }
    # Each method has its own deterministic stream. This deliberately prevents
    # selection-rank pairing across non-matched assets.
    rngs = {
        method: random.Random(f"{seed}:{method}") for method in records_by_method
    }
    for replicate in range(replicates):
        for method, rows in records_by_method.items():
            sampled = stratified_resample(rows, rngs[method])
            if resample_audit is not None and replicate == 0:
                resample_audit.append(
                    (method, tuple(str(row["sample_id"]) for row in sampled))
                )
            sampled_metrics = compute_metrics(sampled)
            for metric in bootstrap_metrics:
                value = sampled_metrics[metric]
                if value is not None:
                    samples[method][metric].append(float(value))

    intervals: dict[str, Any] = {}
    for method in records_by_method:
        intervals[method] = {}
        for metric in bootstrap_metrics:
            values = sorted(samples[method][metric])
            intervals[method][metric] = {
                "estimate": estimates[method][metric],
                "ci95_percentile": (
                    [quantile(values, 0.025), quantile(values, 0.975)]
                    if values
                    else None
                ),
                "valid_replicate_count": len(values),
            }

    differences: dict[str, Any] = {}
    primary = "coverage_weighted_induced_edge_f1_requested"
    for baseline in ("Articraft", "Infinite Mobility"):
        if baseline not in samples or "PV-A" not in samples:
            continue
        values = sorted(
            pva - other
            for pva, other in zip(samples["PV-A"][primary], samples[baseline][primary])
        )
        differences[f"PV-A_minus_{baseline}"] = {
            "metric": primary,
            "estimate": estimates["PV-A"][primary] - estimates[baseline][primary],
            "ci95_percentile": [quantile(values, 0.025), quantile(values, 0.975)],
            "resampling": "independent within-category asset resampling for each method",
            "rank_pairing": False,
        }
    return intervals, differences


def normalize_structure_category(value: str) -> str:
    return "storage_furniture" if value == "storage_furniture_cabinet" else value


def validate_balanced_rows(method: str, rows: list[dict[str, Any]]) -> None:
    if len(rows) != 150:
        raise ValueError(f"{method}: expected 150 requested records, found {len(rows)}")
    ids = [str(row["sample_id"]) for row in rows]
    if len(set(ids)) != len(ids):
        raise ValueError(f"{method}: duplicate sample_id values")
    counts = {category: sum(row["category"] == category for row in rows) for category in CATEGORIES}
    if counts != {category: 30 for category in CATEGORIES}:
        raise ValueError(f"{method}: invalid category counts {counts}")
    if any(row.get("method") != method for row in rows):
        raise ValueError(f"{method}: method labels do not match")
    for row in rows:
        if row.get("valid_tree") and not row.get("available"):
            raise ValueError(f"{method}/{row['sample_id']}: valid but unavailable")
        if row.get("scorable") and not row.get("available"):
            raise ValueError(f"{method}/{row['sample_id']}: scorable but unavailable")


def load_method(method: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    paths = METHOD_PATHS[method]
    root = contained(Path(paths["root"]))
    manifest_path = contained(root / "evaluation_manifest.jsonl")
    structure_path = contained(root / str(paths["structure"]))
    alignment_path = contained(root / str(paths["alignment"]))
    summary_path = contained(root / "summary.json")
    verification_path = contained(root / "verification.json")
    determinism_path = contained(root / "determinism_verification.json")
    verification = load_json(verification_path)
    determinism = load_json(determinism_path)
    if not verification_document_passed(verification):
        raise ValueError(f"{method}: verification gate did not pass")
    if not verification_document_passed(determinism):
        raise ValueError(f"{method}: determinism gate did not pass")
    manifest = load_jsonl(manifest_path)
    structures = load_jsonl(structure_path)
    alignments = load_jsonl(alignment_path)
    manifest_by_id = unique_index(
        manifest, method=method, source="evaluation manifest"
    )
    structure_by_id = unique_index(
        structures, method=method, source="structure records"
    )
    alignment_by_id = unique_index(
        alignments, method=method, source="alignment records"
    )
    if set(alignment_by_id) != set(manifest_by_id):
        raise ValueError(f"{method}: alignment IDs differ from requested manifest")
    if not set(structure_by_id).issubset(manifest_by_id):
        raise ValueError(f"{method}: structure records contain non-manifest IDs")

    rows: list[dict[str, Any]] = []
    for sample_id, manifest_row in manifest_by_id.items():
        alignment = alignment_by_id[sample_id]
        structure = structure_by_id.get(sample_id, {})
        manifest_category = str(manifest_row["category"])
        alignment_category = str(alignment["category"])
        structure_category = normalize_structure_category(
            str(structure.get("category", structure.get("cohort", manifest_category)))
        )
        if len({manifest_category, alignment_category, structure_category}) != 1:
            raise ValueError(f"{method}/{sample_id}: category mismatch")
        available = bool(manifest_row.get("available"))
        if bool(alignment.get("available")) != available:
            raise ValueError(f"{method}/{sample_id}: availability mismatch")
        row = {
            "method": method,
            "sample_id": sample_id,
            "category": manifest_category,
            "available": available,
            "valid_tree": bool(structure.get("valid_tree")),
            "node_count": structure.get("node_count"),
            "semantic_depth": structure.get("semantic_depth"),
            "movable_edge_count": structure.get("movable_edge_count"),
            "scorable": bool(alignment.get("scorable")),
            "semantic_role_coverage": alignment.get("semantic_role_coverage"),
            "induced_parent_child_edge_f1": alignment.get(
                "induced_parent_child_edge_f1"
            ),
            "induced_hierarchy_exact_match": alignment.get(
                "induced_hierarchy_exact_match"
            ),
            "semantic_nesting_accuracy": alignment.get("semantic_nesting_accuracy"),
        }
        if row["valid_tree"] and any(
            row[key] is None
            for key in ("node_count", "semantic_depth", "movable_edge_count")
        ):
            raise ValueError(f"{method}/{sample_id}: valid tree lacks structure metrics")
        rows.append(row)
    validate_balanced_rows(method, rows)
    method_summary = load_json(summary_path)
    expected_structure = method_summary["structure"]["overall"]
    expected_alignment = (
        method_summary["alignment"]["overall"]
        if method == "PV-A"
        else method_summary["alignment"]["summary"]["overall"]
        if method == "Articraft"
        else method_summary["partnet_ontology_alignment"]["overall"]
    )
    observed = compute_metrics(rows)
    consistency_pairs = {
        "available_count": (observed["available_count"], expected_alignment["available_count"]),
        "valid_tree_count": (observed["valid_tree_count"], expected_structure["valid_tree_count"]),
        "node_count_mean_pooled_valid": (
            observed["node_count_mean_pooled_valid"],
            expected_structure["node_count_mean"],
        ),
        "semantic_depth_mean_pooled_valid": (
            observed["semantic_depth_mean_pooled_valid"],
            expected_structure["semantic_depth_mean"],
        ),
        "movable_joint_count_mean_pooled_valid": (
            observed["movable_joint_count_mean_pooled_valid"],
            expected_structure["movable_edge_count_mean"],
        ),
        "role_coverage_requested": (
            observed["role_coverage_requested"],
            expected_alignment["semantic_role_coverage_requested_macro"],
        ),
        "scorable_requested": (
            observed["scorable_requested"],
            expected_alignment["scorable_asset_coverage_requested"],
        ),
        "coverage_weighted_induced_edge_f1_requested": (
            observed["coverage_weighted_induced_edge_f1_requested"],
            expected_alignment["coverage_weighted_induced_edge_f1_requested_macro"],
        ),
        "induced_edge_f1_requested": (
            observed["induced_edge_f1_requested"],
            expected_alignment["parent_child_edge_f1_requested_macro"],
        ),
        "induced_exact_requested": (
            observed["induced_exact_requested"],
            expected_alignment["hierarchy_exact_match_requested_rate"],
        ),
        "semantic_parent_alignment_requested": (
            observed["semantic_parent_alignment_requested"],
            expected_alignment["semantic_nesting_accuracy_requested_macro"],
        ),
    }
    for metric, (actual, expected) in consistency_pairs.items():
        if actual is None or expected is None or abs(float(actual) - float(expected)) > 1e-12:
            raise ValueError(
                f"{method}: recomputed {metric}={actual!r} differs from method summary {expected!r}"
            )
    evidence = {
        "root": str(root),
        "manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)},
        "structure_records": {"path": str(structure_path), "sha256": sha256(structure_path)},
        "alignment_records": {"path": str(alignment_path), "sha256": sha256(alignment_path)},
        "method_summary": {"path": str(summary_path), "sha256": sha256(summary_path)},
        "method_verification": {
            "path": str(verification_path),
            "sha256": sha256(verification_path),
            "passed": True,
        },
        "method_determinism": {
            "path": str(determinism_path),
            "sha256": sha256(determinism_path),
            "passed": True,
        },
        "method_summary_consistency": {
            "status": "PASS",
            "checked_metrics": sorted(consistency_pairs),
            "absolute_tolerance": 1e-12,
        },
    }
    return rows, evidence


def load_separate_panels() -> tuple[dict[str, Any], dict[str, Any]]:
    lam_summary_path = contained(RUNTIME_ROOT / "lam/summary.json")
    lam_validation_path = contained(RUNTIME_ROOT / "lam/validation.json")
    lam = load_json(lam_summary_path)
    lam_validation = load_json(lam_validation_path)
    if not verification_document_passed(lam_validation):
        raise ValueError("LAM: validation gate did not pass")
    structure_path = contained(
        RUNTIME_ROOT / "lam/official_release_full_strict/structure_summary.json"
    )
    alignment_path = contained(
        RUNTIME_ROOT / "lam/official_release_full_strict/partnet_summary.json"
    )
    lam_structure = load_json(structure_path)
    lam_alignment = load_json(alignment_path)
    lam_panel = {
        "panel_role": "unbalanced official-release supplementary audit; excluded from generated-method ranking",
        "ranking_eligible": False,
        "requested_count": lam["official_release_full_strict"]["requested_count"],
        "category_counts": lam["official_release_full_strict"]["candidate_counts"],
        "status": lam["official_release_full_strict"]["status"],
        "balanced_n150_status": lam["balanced_n150_intention"]["status"],
        "category_macro_structure": lam_structure["category_macro"],
        "category_macro_alignment": lam_alignment["category_macro"],
        "category_macro_bootstrap": lam_alignment[
            "bootstrap_intervals_category_macro"
        ],
        "input_evidence": {
            "summary_sha256": sha256(lam_summary_path),
            "validation_sha256": sha256(lam_validation_path),
            "validation_passed": True,
            "structure_summary_sha256": sha256(structure_path),
            "alignment_summary_sha256": sha256(alignment_path),
        },
    }

    artiverse_path = contained(RUNTIME_ROOT / "artiverse_reference/summary.json")
    artiverse_verification_path = contained(
        RUNTIME_ROOT / "artiverse_reference/verification.json"
    )
    artiverse_determinism_path = contained(
        RUNTIME_ROOT / "artiverse_reference/determinism_verification.json"
    )
    artiverse = load_json(artiverse_path)
    artiverse_verification = load_json(artiverse_verification_path)
    artiverse_determinism = load_json(artiverse_determinism_path)
    if not verification_document_passed(artiverse_verification):
        raise ValueError("Artiverse: verification gate did not pass")
    if not verification_document_passed(artiverse_determinism):
        raise ValueError("Artiverse: determinism gate did not pass")
    artiverse_panel = {
        "panel_role": artiverse["role"],
        "ranking_eligible": False,
        "paper_ready": artiverse["paper_ready"],
        "strict_five_category_exact_panel_feasible": artiverse[
            "strict_five_category_exact_panel_feasible"
        ],
        "primary_4class_matched_overlap": artiverse["primary"],
        "table_alias_5class_sensitivity": artiverse["table_alias_sensitivity"],
        "input_evidence": {
            "summary_sha256": sha256(artiverse_path),
            "verification_sha256": sha256(artiverse_verification_path),
            "verification_passed": True,
            "determinism_sha256": sha256(artiverse_determinism_path),
            "determinism_passed": True,
        },
    }
    return lam_panel, artiverse_panel


def fmt_percent(value: float | None) -> str:
    return "N/A" if value is None else f"{100 * value:.1f}"


def fmt_ci(entry: dict[str, Any]) -> str:
    low, high = entry["ci95_percentile"]
    return f"{fmt_percent(entry['estimate'])} [{fmt_percent(low)}, {fmt_percent(high)}]"


def fmt_scalar_ci(entry: dict[str, Any]) -> str:
    low, high = entry["ci95_percentile"]
    return f"{entry['estimate']:.2f} [{low:.2f}, {high:.2f}]"


def fmt_percent_ci_markdown(entry: dict[str, Any]) -> str:
    low, high = entry["ci95_percentile"]
    return (
        f"{100 * entry['estimate']:.1f}% "
        f"[{100 * low:.1f}, {100 * high:.1f}]"
    )


def fmt_scalar_ci_markdown(entry: dict[str, Any]) -> str:
    low, high = entry["ci95_percentile"]
    return f"{entry['estimate']:.3f} [{low:.3f}, {high:.3f}]"


def validate_expected_fragments(text: str, expected: dict[str, str]) -> int:
    missing = [label for label, fragment in expected.items() if fragment not in text]
    if missing:
        raise ValueError(
            "paper Markdown missing or stale fragments: " + ", ".join(missing)
        )
    return len(expected)


def expected_paper_fragments(payload: dict[str, Any]) -> dict[str, str]:
    panel = payload["balanced_generated_method_panel"]
    labels = {
        "PV-A": "**PV-A (ours)**",
        "Articraft": "**Articraft**",
        "Infinite Mobility": "**Infinite Mobility**",
    }
    fragments: dict[str, str] = {}
    for method in METHODS:
        item = panel["methods"][method]
        point = item["point_estimates"]
        boot = item["bootstrap_intervals"]
        requested = point["requested_count"]
        available = point["available_count"]
        valid = point["valid_tree_count"]
        fragments[f"{method} structural row"] = (
            f"| {labels[method]} | {available}/{requested} | "
            f"{valid}/{available} ({100 * point['valid_available']:.1f}%) | "
            f"{valid}/{requested} ({100 * point['valid_requested']:.1f}%) | "
            f"{fmt_scalar_ci_markdown(boot['node_count_mean_valid'])} | "
            f"{fmt_scalar_ci_markdown(boot['semantic_depth_mean_valid'])} | "
            f"{fmt_scalar_ci_markdown(boot['movable_joint_count_mean_valid'])} |"
        )
        fragments[f"{method} alignment row"] = (
            f"| {labels[method]} | "
            f"{fmt_percent_ci_markdown(boot['role_coverage_requested'])} | "
            f"{fmt_percent_ci_markdown(boot['scorable_requested'])} | "
            f"{fmt_percent_ci_markdown(boot['coverage_weighted_induced_edge_f1_requested'])} | "
            f"{fmt_percent_ci_markdown(boot['induced_edge_f1_requested'])} | "
            f"{fmt_percent_ci_markdown(boot['induced_exact_requested'])} | "
            f"{fmt_percent_ci_markdown(boot['semantic_parent_alignment_requested'])} |"
        )

    for baseline in ("Articraft", "Infinite Mobility"):
        difference = panel["pairwise_primary_metric_differences"][
            f"PV-A_minus_{baseline}"
        ]
        low, high = difference["ci95_percentile"]
        fragments[f"PV-A minus {baseline} difference"] = (
            f"PV-A minus {baseline} is "
            f"{100 * difference['estimate']:+.2f} percentage points "
            f"(95% CI [{100 * low:+.2f}, {100 * high:+.2f}])"
        )

    lam = payload["separate_nonranking_panels"][
        "LAM_official_release_N300_unbalanced"
    ]
    lam_alignment = lam["category_macro_alignment"]
    lam_ci = lam["category_macro_bootstrap"]["metrics"][
        "coverage_weighted_induced_edge_f1_requested"
    ]["ci95_percentile"]
    fragments["LAM supplementary row"] = (
        f"| LAM full strict, severely unbalanced | {lam['requested_count']} | "
        f"{100 * lam['category_macro_structure']['valid_tree_rate_requested']:.1f}% | "
        f"{100 * lam_alignment['semantic_role_coverage_requested_macro']:.1f}% | "
        f"{100 * lam_alignment['scorable_asset_coverage_requested']:.1f}% | "
        f"{100 * lam_alignment['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% "
        f"[{100 * lam_ci[0]:.1f}, {100 * lam_ci[1]:.1f}] |"
    )

    artiverse = payload["separate_nonranking_panels"][
        "Artiverse_curated_data_reference"
    ]
    for panel_key, label in (
        ("primary_4class_matched_overlap", "Artiverse primary four-category reference"),
        (
            "table_alias_5class_sensitivity",
            "Artiverse `coffee_table` alias sensitivity only",
        ),
    ):
        reference = artiverse[panel_key]
        structure = reference["structure"]
        overall = structure["overall"]
        macro = structure["category_macro_structure"]
        alignment = reference["urdf_name_only_ontology_alignment_sensitivity"]
        alignment_ci = reference["urdf_name_only_alignment_bootstrap"][
            "ci95_percentile"
        ]
        fragments[f"{label} row"] = (
            f"| {label} | {overall['available_count']}/{reference['requested_count']} | "
            f"{overall['valid_tree_count']}/{reference['requested_count']} "
            f"({100 * overall['rates']['valid_requested']:.1f}%) | "
            f"{macro['semantic_depth_mean_valid_only']:.3f} | "
            f"{macro['all_nonfixed_joint_count_mean_valid']:.3f} | "
            f"{100 * alignment['semantic_role_coverage_requested_macro']:.1f}% | "
            f"{alignment['scorable_count']}/{reference['requested_count']} "
            f"({100 * alignment['scorable_asset_coverage_requested']:.1f}%) | "
            f"{100 * alignment['coverage_weighted_induced_edge_f1_requested_macro']:.1f}% "
            f"[{100 * alignment_ci[0]:.1f}, {100 * alignment_ci[1]:.1f}] |"
        )

    fragments.update(
        {
            "coverage-weighted formula": (
                r"S_m = \frac{1}{150}\sum_{i=1}^{150} c_i f_i,"
            ),
            "LAM blocked boundary": "`BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT`",
            "Artiverse nonmethod boundary": (
                "Artiverse is a curated pre-release dataset reference, not a generation method."
            ),
            "ontology proxy boundary": (
                "PartNet-based measurements are ontology alignment proxies, not instance-level parent--child annotations."
            ),
        }
    )
    return fragments


def make_report(payload: dict[str, Any]) -> str:
    lines = [
        "# Expanded hierarchy aggregation (balanced N=150 method panel)",
        "",
        "The generated-method panel contains five frozen categories and 30 requested assets per category. Failures remain in every requested-denominator metric. Structure means are equal-category macros of within-category means conditional on valid trees.",
        "",
        "| Method | Available / requested | Valid tree / requested | Valid tree / available | Nodes (95% CI) | Kinematic tree depth (95% CI) | Movable joints (95% CI) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for method in METHODS:
        item = payload["balanced_generated_method_panel"]["methods"][method]
        point = item["point_estimates"]
        boot = item["bootstrap_intervals"]
        lines.append(
            f"| {method} | {point['available_count']}/150 | {point['valid_tree_count']}/150 | "
            f"{fmt_percent(point['valid_available'])} | {fmt_scalar_ci(boot['node_count_mean_valid'])} | "
            f"{fmt_scalar_ci(boot['semantic_depth_mean_valid'])} | "
            f"{fmt_scalar_ci(boot['movable_joint_count_mean_valid'])} |"
        )
    lines.extend(
        [
            "",
            "| Method | Role coverage % | Scorable % | CW induced Edge F1 % (95% CI) | Requested induced Edge F1 % | Exact % | Semantic-parent alignment % |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for method in METHODS:
        item = payload["balanced_generated_method_panel"]["methods"][method]
        boot = item["bootstrap_intervals"]
        lines.append(
            f"| {method} | {fmt_percent(boot['role_coverage_requested']['estimate'])} | "
            f"{fmt_percent(boot['scorable_requested']['estimate'])} | "
            f"{fmt_ci(boot['coverage_weighted_induced_edge_f1_requested'])} | "
            f"{fmt_percent(boot['induced_edge_f1_requested']['estimate'])} | "
            f"{fmt_percent(boot['induced_exact_requested']['estimate'])} | "
            f"{fmt_percent(boot['semantic_parent_alignment_requested']['estimate'])} |"
        )
    lines.extend(["", "Independent primary-metric differences:", ""])
    for label, item in payload["balanced_generated_method_panel"][
        "pairwise_primary_metric_differences"
    ].items():
        low, high = item["ci95_percentile"]
        lines.append(
            f"- {label}: {100 * item['estimate']:.1f} pp "
            f"[{100 * low:.1f}, {100 * high:.1f}]."
        )
    lines.extend(
        [
            "",
            "LAM is reported only as an unbalanced N=300 official-release supplement; its balanced N=150 intention is blocked and has no ranking score. Artiverse is a curated released-dataset reference, not a generation method; its four-category matched-overlap panel and coffee-table alias sensitivity panel are excluded from the method ranking.",
            "",
            "All ontology measurements are PartNet-ontology-referenced induced semantic alignment proxies, not instance-level kinematic correctness.",
        ]
    )
    return "\n".join(lines) + "\n"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="Write an explicit incomplete status instead of failing when a method input is missing/incomplete.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Validate inputs and compute 32 bootstrap replicates without writing outputs.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    records_by_method: dict[str, list[dict[str, Any]]] = {}
    evidence: dict[str, Any] = {}
    errors: dict[str, str] = {}
    for method in METHODS:
        try:
            records_by_method[method], evidence[method] = load_method(method)
        except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
            errors[method] = str(exc)

    if errors:
        if not args.allow_incomplete:
            raise RuntimeError(f"expanded hierarchy inputs incomplete: {errors}")
        payload = {
            "protocol_id": "nano3d_hierarchy_expanded_n150_aggregate_v1",
            "status": "INCOMPLETE_NOT_A_PAPER_RESULT",
            "errors": errors,
            "ranking_eligible": False,
        }
        if args.self_test:
            print(json.dumps(payload, indent=2, sort_keys=True))
            return 2
        output = contained(OUTPUT, exists=False)
        output.mkdir(parents=True, exist_ok=True)
        (output / "summary.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return 2

    intervals, differences = bootstrap_methods(
        records_by_method,
        replicates=32 if args.self_test else BOOTSTRAP_REPLICATES,
        seed=BOOTSTRAP_SEED,
    )
    lam_panel, artiverse_panel = load_separate_panels()
    methods = {
        method: {
            "point_estimates": compute_metrics(records_by_method[method]),
            "bootstrap_intervals": intervals[method],
            "input_evidence": evidence[method],
        }
        for method in METHODS
    }
    payload = {
        "protocol_id": "nano3d_hierarchy_expanded_n150_aggregate_v1",
        "status": "SELF_TEST_PASS" if args.self_test else "COMPLETE",
        "claim_boundary": "Package-level structural validity and PartNet-ontology-referenced induced semantic alignment; not instance-level kinematic correctness.",
        "balanced_generated_method_panel": {
            "ranking_eligible": True,
            "methods_in_ranking": list(METHODS),
            "categories": CATEGORIES,
            "requested_per_category": 30,
            "requested_per_method": 150,
            "failures_replaced": False,
            "structure_conditioning": "node, depth, and movable-joint means are equal-category macros of within-category valid-tree means; pooled-valid values are diagnostic only",
            "bootstrap": {
                "design": "independent category-stratified asset bootstrap with replacement",
                "replicates": 32 if args.self_test else BOOTSTRAP_REPLICATES,
                "seed": BOOTSTRAP_SEED,
                "interval": "95% percentile",
                "category_weighting": "five categories equally weighted; 30 draws per category",
                "cross_method_pairing": "none; methods use separate deterministic random streams and are independently resampled within category",
                "pairwise_interval_multiplicity": "unadjusted exploratory intervals",
            },
            "methods": methods,
            "pairwise_primary_metric_differences": differences,
        },
        "separate_nonranking_panels": {
            "LAM_official_release_N300_unbalanced": lam_panel,
            "Artiverse_curated_data_reference": artiverse_panel,
        },
    }
    validation = {
        "status": "PASS",
        "check_count": 0,
        "checks": [],
    }
    checks: list[tuple[str, Callable[[], bool]]] = [
        ("ranking contains exactly PV-A, Articraft, Infinite Mobility", lambda: set(methods) == set(METHODS)),
        ("each ranking method has 150 requested assets", lambda: all(methods[m]["point_estimates"]["requested_count"] == 150 for m in METHODS)),
        ("each ranking method has 30 assets per category", lambda: all(all(sum(r["category"] == c for r in records_by_method[m]) == 30 for c in CATEGORIES) for m in METHODS)),
        ("LAM is excluded from ranking", lambda: not lam_panel["ranking_eligible"]),
        ("Artiverse is excluded from ranking", lambda: not artiverse_panel["ranking_eligible"]),
        ("pairwise differences are unpaired", lambda: all(not entry["rank_pairing"] for entry in differences.values())),
        ("both requested pairwise differences exist", lambda: set(differences) == {"PV-A_minus_Articraft", "PV-A_minus_Infinite Mobility"}),
    ]
    for name, check in checks:
        passed = bool(check())
        validation["checks"].append({"name": name, "passed": passed})
        if not passed:
            validation["status"] = "FAIL"
    validation["check_count"] = len(validation["checks"])
    if validation["status"] != "PASS":
        raise RuntimeError(f"aggregate validation failed: {validation}")
    if args.self_test:
        print(json.dumps(validation, indent=2, sort_keys=True))
        return 0

    paper_path = contained(PAPER_MARKDOWN)
    paper_text = paper_path.read_text(encoding="utf-8")
    paper_fragment_count = validate_expected_fragments(
        paper_text, expected_paper_fragments(payload)
    )
    validation["checks"].append(
        {
            "name": "paper Markdown matches aggregate-derived values and claim boundaries",
            "passed": True,
        }
    )
    validation["check_count"] = len(validation["checks"])
    validation["paper_markdown"] = {
        "path": str(paper_path),
        "sha256": sha256(paper_path),
        "validated_fragment_count": paper_fragment_count,
    }

    output = contained(OUTPUT, exists=False)
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "summary.json"
    summary_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    report_path = output / "report.md"
    report_path.write_text(make_report(payload), encoding="utf-8")
    validation["summary_sha256"] = sha256(summary_path)
    validation["report_sha256"] = sha256(report_path)
    validation["aggregator_sha256"] = sha256(Path(__file__).resolve())
    (output / "verification.json").write_text(
        json.dumps(validation, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "status": payload["status"],
                "output": str(output),
                "pairwise_primary_metric_differences": differences,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
