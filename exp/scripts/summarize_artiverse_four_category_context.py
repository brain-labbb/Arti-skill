#!/usr/bin/env python3
"""Reaggregate existing generated-method records on Artiverse's four categories."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
from typing import Any

from hierarchy_extended_metrics import (
    aggregate as aggregate_structure,
    analyze_urdf,
    topology_consistency,
)
from partnet_hierarchy_correctness import aggregate


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_ARTIVERSE_SUMMARY = (
    EXP_ROOT / "runtime/nano3d_hierarchy_correctness/artiverse_reference/summary.json"
)
DEFAULT_OUTPUT = (
    EXP_ROOT
    / "runtime/nano3d_hierarchy_correctness/artiverse_reference/four_category_generated_method_context.json"
)
SOURCE_RECORDS = {
    "PV-A": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva/partnet_scores/records.jsonl",
    "LAM": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/lam/partnet_scores/records.jsonl",
    "Articraft": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/articraft/partnet_scores/records.jsonl",
    "Infinite Mobility": EXP_ROOT
    / "runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel/partnet_scores/records.jsonl",
}
SOURCE_MANIFESTS = {
    "PV-A": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva/evaluation_manifest.jsonl",
    "LAM": EXP_ROOT / "runtime/nano3d_hierarchy_correctness/lam/evaluation_manifest.jsonl",
    "Articraft": EXP_ROOT
    / "runtime/nano3d_hierarchy_correctness/articraft/evaluation_manifest.jsonl",
    "Infinite Mobility": EXP_ROOT
    / "runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel/evaluation_manifest.jsonl",
}
CATEGORIES = ("storage_furniture", "refrigerator", "dishwasher", "microwave")
BOOTSTRAP_REPLICATES = 10_000
BOOTSTRAP_SEED = 20260812


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def score(row: dict[str, Any]) -> float:
    return float(row.get("parent_child_edge_f1") or 0.0) * float(
        row.get("semantic_role_coverage") or 0.0
    )


def quantile(values: list[float], p: float) -> float:
    position = (len(values) - 1) * p
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    weight = position - lower
    return values[lower] * (1 - weight) + values[upper] * weight


def bootstrap(rows: list[dict[str, Any]], method_index: int) -> dict[str, Any]:
    grouped = {category: [row for row in rows if row["category"] == category] for category in CATEGORIES}
    if any(len(records) != 6 for records in grouped.values()):
        raise ValueError("context bootstrap requires six existing records per category")
    rng = random.Random(BOOTSTRAP_SEED + method_index)
    samples = []
    for _ in range(BOOTSTRAP_REPLICATES):
        sampled = [rng.choice(grouped[category]) for category in CATEGORIES for _ in range(6)]
        samples.append(sum(score(row) for row in sampled) / 24)
    samples.sort()
    return {
        "estimate": sum(score(row) for row in rows) / 24,
        "ci95_percentile": [quantile(samples, 0.025), quantile(samples, 0.975)],
        "replicates": BOOTSTRAP_REPLICATES,
        "seed": BOOTSTRAP_SEED + method_index,
        "design": "four-category stratified bootstrap; six draws with replacement per method/category",
    }


def structure_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    structure_rows = []
    per_category = {}
    for row in rows:
        structure_row = {
            "method": row.get("method"),
            "sample_id": row.get("sample_id"),
            "category": row["category"],
            "available": bool(row.get("available")),
            "urdf_path": row.get("urdf_path"),
            "urdf_sha256": row.get("urdf_sha256"),
            "evaluation_complete": False,
        }
        if structure_row["available"] and structure_row["urdf_path"]:
            try:
                urdf_path = contained(Path(str(structure_row["urdf_path"])))
                if sha256_file(urdf_path) != structure_row["urdf_sha256"]:
                    raise ValueError("URDF SHA-256 differs from frozen record")
                structure_row.update(analyze_urdf(urdf_path))
                structure_row["evaluation_complete"] = True
                structure_row["evaluation_error"] = None
            except Exception as exc:
                structure_row["evaluation_error"] = f"{type(exc).__name__}: {exc}"
        else:
            structure_row["evaluation_error"] = "unavailable frozen URDF"
        structure_rows.append(structure_row)

    for category in CATEGORIES:
        category_rows = [row for row in structure_rows if row["category"] == category]
        evaluated = [row for row in category_rows if row["evaluation_complete"]]
        valid = [row for row in evaluated if row.get("valid_tree")]
        per_category[category] = {
            "requested_count": 6,
            "available_count": sum(bool(row["available"]) for row in category_rows),
            "parsed_count": len(evaluated),
            "valid_tree_count": len(valid),
            "metrics": aggregate_structure(evaluated, requested_count=6),
            "topology_consistency": topology_consistency(valid),
        }
    evaluated = [row for row in structure_rows if row["evaluation_complete"]]
    valid = [row for row in evaluated if row.get("valid_tree")]
    topology_fields = (
        "unique_signature_rate",
        "mode_rate",
        "pairwise_exact_rate",
        "normalized_entropy",
    )
    category_macro_topology = {}
    for field in topology_fields:
        values = [
            per_category[category]["topology_consistency"][field]
            for category in CATEGORIES
            if per_category[category]["topology_consistency"][field] is not None
        ]
        category_macro_topology[field] = (
            sum(float(value) for value in values) / len(values) if values else None
        )
    available_count = sum(bool(row["available"]) for row in structure_rows)
    return {
        "records": structure_rows,
        "overall": {
            "requested_count": 24,
            "available_count": available_count,
            "parsed_count": len(evaluated),
            "valid_tree_count": len(valid),
            "valid_tree_rate_available": (
                len(valid) / available_count if available_count else None
            ),
            "valid_tree_rate_requested": len(valid) / 24,
            "metrics": aggregate_structure(evaluated, requested_count=24),
        },
        "per_category": per_category,
        "category_macro_topology_consistency": category_macro_topology,
        "pooled_topology_consistency_diagnostic": topology_consistency(valid),
        "conditioning": "descriptive structure means are conditional on valid frozen URDF trees",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artiverse-summary", type=Path, default=DEFAULT_ARTIVERSE_SUMMARY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    artiverse_summary_path = contained(args.artiverse_summary)
    output = contained(args.output, exists=False)
    artiverse = json.loads(artiverse_summary_path.read_text(encoding="utf-8"))
    if artiverse["primary"]["categories"] != list(CATEGORIES):
        raise ValueError("Artiverse primary categories differ from frozen context categories")

    methods = {}
    for method_index, (method, raw_path) in enumerate(SOURCE_RECORDS.items()):
        path = contained(raw_path)
        manifest_path = contained(SOURCE_MANIFESTS[method])
        rows = [row for row in read_jsonl(path) if row.get("category") in CATEGORIES]
        if len(rows) != 24:
            raise ValueError(f"{method} does not have exactly 24 frozen four-category records")
        if any(sum(row["category"] == category for row in rows) != 6 for category in CATEGORIES):
            raise ValueError(f"{method} category counts are not 6 each")
        methods[method] = {
            "aggregate": aggregate(rows),
            "bootstrap": bootstrap(rows, method_index),
            "structure": structure_summary(rows),
            "source_records_path": str(path),
            "source_records_sha256": sha256_file(path),
            "source_cohort_manifest_path": str(manifest_path),
            "source_cohort_manifest_sha256": sha256_file(manifest_path),
        }

    result = {
        "protocol_id": "artiverse_four_category_generated_method_context_v1",
        "categories": list(CATEGORIES),
        "requested_per_category_per_method": 6,
        "requested_per_method": 24,
        "interpretation": (
            "Separate matched-category context reaggregated from each generated method's existing frozen records. "
            "Assets are not instance-paired across methods or with Artiverse."
        ),
        "ranking_policy": (
            "Artiverse is a real-data reference and is excluded from generated-method rankings, pairwise differences, and significance claims."
        ),
        "artiverse_summary_path": str(artiverse_summary_path),
        "artiverse_summary_sha256": sha256_file(artiverse_summary_path),
        "methods": methods,
        "pairwise_differences": None,
    }
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "methods": list(methods)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
