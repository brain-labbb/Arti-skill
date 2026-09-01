#!/usr/bin/env python3
"""Compute extended Table 3 diagnostics for the frozen PV-A matched cohort."""

from __future__ import annotations

from collections import defaultdict
import hashlib
import json
from pathlib import Path
from typing import Any

from hierarchy_extended_metrics import aggregate, analyze_urdf, topology_consistency


EXP_ROOT = Path(__file__).resolve().parents[1]
INPUT = EXP_ROOT / "runtime/nano3d_hierarchy_paper/pva"
OUTPUT = INPUT / "extended"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    source_rows = json.loads((INPUT / "records.json").read_text(encoding="utf-8"))
    records: list[dict[str, Any]] = []
    for source in source_rows:
        row = {
            "category": source["category"],
            "seed": source["seed"],
            "compile_status": source["compile_status"],
            "evaluated": False,
        }
        artifact_dir = source.get("artifact_dir")
        if artifact_dir:
            urdf = Path(str(artifact_dir)) / "model.urdf"
            if urdf.is_file():
                row.update(analyze_urdf(urdf))
                row["evaluated"] = True
                row["model_urdf"] = str(urdf)
                row["model_urdf_sha256"] = sha256(urdf)
        records.append(row)

    by_category: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in records:
        by_category[str(row["category"])].append(row)
    per_category = {}
    for category, rows in sorted(by_category.items()):
        evaluated = [row for row in rows if row["evaluated"]]
        per_category[category] = {
            "metrics": aggregate(evaluated, requested_count=len(rows)),
            "topology_consistency": topology_consistency(evaluated),
        }
    evaluated = [row for row in records if row["evaluated"]]
    summary = {
        "protocol_id": "nano3d_hierarchy_extended_representation_v1",
        "method": "PV-A",
        "requested_count": len(records),
        "overall": aggregate(evaluated, requested_count=len(records)),
        "per_category": per_category,
        "category_macro_topology_consistency": {
            field: sum(
                float(item["topology_consistency"][field])
                for item in per_category.values()
                if item["topology_consistency"][field] is not None
            )
            / sum(
                item["topology_consistency"][field] is not None
                for item in per_category.values()
            )
            for field in ("unique_signature_rate", "mode_rate", "pairwise_exact_rate", "normalized_entropy")
        },
        "correctness_metrics": {
            "parent_child_edge_f1": None,
            "hierarchy_exact_match": None,
            "semantic_nesting_accuracy": None,
            "reason": "No independent asset-specific hierarchy gold; generated URDF edges are not used as their own reference.",
        },
        "source_records_sha256": sha256(INPUT / "records.json"),
        "evaluator_sha256": sha256(Path(__file__).with_name("hierarchy_extended_metrics.py")),
    }
    OUTPUT.mkdir(parents=True, exist_ok=True)
    (OUTPUT / "records.json").write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (OUTPUT / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
