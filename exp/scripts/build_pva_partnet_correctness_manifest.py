#!/usr/bin/env python3
"""Adapt the frozen PV-A correctness cohort to the shared scorer manifest."""

from __future__ import annotations

import json
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
SOURCE = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva/records.json"
OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/pva/evaluation_manifest.jsonl"
EXPECTED_CATEGORIES = {
    "storage_furniture",
    "table",
    "refrigerator",
    "dishwasher",
    "microwave",
}


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def main() -> int:
    source = contained(SOURCE)
    records = json.loads(source.read_text(encoding="utf-8"))
    if len(records) != 30:
        raise ValueError(f"expected 30 frozen records, found {len(records)}")

    rows = []
    for record in sorted(records, key=lambda row: (str(row["category"]), int(row["seed"]))):
        category = str(record["category"])
        seed = int(record["seed"])
        available = bool(record.get("parseable_final_urdf"))
        row = {
            "method": "PV-A",
            "sample_id": f"{category}/seed_{seed}",
            "category": category,
            "urdf_path": record.get("model_urdf") if available else None,
            "urdf_sha256": record.get("model_urdf_sha256") if available else None,
            "available": available,
            "selection_rank": seed,
        }
        if not available:
            row["unavailable_reason"] = (
                f"{record.get('failure_type', 'compile failure')}: "
                f"{record.get('failure_details', 'no final URDF')}"
            )
        rows.append(row)

    counts = {
        category: sum(row["category"] == category for row in rows)
        for category in EXPECTED_CATEGORIES
    }
    if set(counts.values()) != {6} or {row["category"] for row in rows} != EXPECTED_CATEGORIES:
        raise ValueError(f"invalid frozen category counts: {counts}")
    for row in rows:
        if row["available"]:
            contained(Path(str(row["urdf_path"])))

    output = contained(OUTPUT, exists=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )
    print(json.dumps({"output": str(output), "rows": len(rows), "counts": counts}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
