#!/usr/bin/env python3
"""Aggregate atomic asset records into the frozen nine-metric table."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
from typing import Any, Mapping, Sequence

import numpy as np


def _finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _manifest_identities(path: Path) -> dict[tuple[str, str], dict[str, Any]]:
    rows: dict[tuple[str, str], dict[str, Any]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        identity = (str(row["dataset_slug"]), str(row["asset_id"]))
        if identity in rows:
            raise ValueError(f"duplicate manifest identity: {identity}")
        rows[identity] = row
    return rows


def _records(root: Path) -> dict[tuple[str, str], dict[str, Any]]:
    rows: dict[tuple[str, str], dict[str, Any]] = {}
    for path in sorted((root / "records").glob("*/*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        identity_payload = record.get("identity", {})
        identity = (str(identity_payload.get("dataset_slug", "")), str(identity_payload.get("asset_id", "")))
        if not all(identity):
            raise ValueError(f"record lacks identity: {path}")
        if identity in rows:
            raise ValueError(f"duplicate record identity: {identity}")
        rows[identity] = record
    return rows


def _mean(values: Sequence[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _percent(value: float | None) -> float | None:
    return 100.0 * value if value is not None else None


def _aggregate_dataset(
    expected: Sequence[tuple[str, str]], records: Mapping[tuple[str, str], Mapping[str, Any]]
) -> dict[str, Any]:
    available = [records[identity] for identity in expected if identity in records]
    structural = [
        row["structural"]
        for row in available
        if isinstance(row.get("structural"), dict) and row["structural"].get("status") == "complete"
    ]
    collision = [
        row["collision"]
        for row in available
        if isinstance(row.get("collision"), dict) and row["collision"].get("status") == "complete"
    ]
    support_rates = [
        float(row["static_joint_support"]["rate"])
        for row in structural
        if _finite(row.get("static_joint_support", {}).get("rate"))
    ]
    gap_values = [
        float(row["static_joint_gap_fraction_p95"])
        for row in structural
        if _finite(row.get("static_joint_gap_fraction_p95"))
    ]
    axis_rates = [
        float(row["static_axis_support"]["rate"])
        for row in structural
        if _finite(row.get("static_axis_support", {}).get("rate"))
    ]
    axis_pose_rates = [
        float(row["pose_swept"]["axis_pose_rate"])
        for row in structural
        if _finite(row.get("pose_swept", {}).get("axis_pose_rate"))
    ]
    motion_ranges = [
        float(row["collision_free_joint_motion_range_rate"])
        for row in collision
        if _finite(row.get("collision_free_joint_motion_range_rate"))
    ]
    premature_rates = [
        float(row["premature_collision_free_joint_rate"])
        for row in collision
        if _finite(row.get("premature_collision_free_joint_rate"))
    ]
    growth_values = [
        float(row["penetration_growth_p95_fraction"])
        for row in collision
        if _finite(row.get("penetration_growth_p95_fraction"))
    ]
    expected_count = len(expected)
    return {
        "expected_assets": expected_count,
        "recorded_assets": len(available),
        "record_coverage_percentage": 100 * len(available) / expected_count if expected_count else None,
        "structural_evaluable_assets": len(structural),
        "structural_coverage_percentage": 100 * len(structural) / expected_count if expected_count else None,
        "collision_evaluable_assets": len(collision),
        "collision_coverage_percentage": 100 * len(collision) / expected_count if expected_count else None,
        "rooted_assets_percentage": (
            100 * sum(row.get("rooted_asset") is True for row in structural) / expected_count
            if expected_count
            else None
        ),
        "joint_support_macro_percentage": _percent(_mean(support_rates)),
        "joint_gap_p95_percent_diag": (
            100 * float(np.percentile(gap_values, 95)) if gap_values else None
        ),
        "axis_rooted_assets_percentage": (
            100 * sum(row.get("axis_rooted_asset") is True for row in structural) / expected_count
            if expected_count
            else None
        ),
        "axis_support_macro_percentage": _percent(_mean(axis_rates)),
        "k9_axis_pose_support_macro_percentage": _percent(_mean(axis_pose_rates)),
        "collision_free_joint_motion_range_macro_percentage": _percent(_mean(motion_ranges)),
        "premature_collision_free_joints_macro_percentage": _percent(_mean(premature_rates)),
        "penetration_growth_asset_balanced_p95_percent_diag": (
            100 * float(np.percentile(growth_values, 95)) if growth_values else None
        ),
        "denominator_policy": {
            "asset_binary_metrics": "full manifest denominator; missing/not-evaluable cannot improve pass rate",
            "continuous_metrics": "evaluable-asset macro with explicit coverage",
            "p95_metrics": "P95 across one per-asset P95 value, preventing multi-joint assets from dominating",
        },
    }


def _display(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--datasets", default="all")
    args = parser.parse_args(argv)
    expected_rows = _manifest_identities(args.manifest)
    records = _records(args.results)
    selected = None if args.datasets == "all" else {x.strip() for x in args.datasets.split(",") if x.strip()}
    by_dataset: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for identity in expected_rows:
        if selected is None or identity[0] in selected:
            by_dataset[identity[0]].append(identity)
    unexpected = sorted(identity for identity in records if identity not in expected_rows)
    summaries = {
        slug: _aggregate_dataset(identities, records)
        for slug, identities in sorted(by_dataset.items())
    }
    summary = {
        "schema_version": "articulated_integrity_aggregate_v1",
        "manifest": str(args.manifest.resolve()),
        "results": str(args.results.resolve()),
        "unexpected_record_count": len(unexpected),
        "unexpected_record_examples": [list(item) for item in unexpected[:20]],
        "datasets": summaries,
    }
    args.out.mkdir(parents=True, exist_ok=True)
    _atomic_text(args.out / "summary.json", json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True) + "\n")
    columns = [
        ("dataset", None),
        ("N", "expected_assets"),
        ("Rooted Assets (%) up", "rooted_assets_percentage"),
        ("Joint Support macro (%) up", "joint_support_macro_percentage"),
        ("Joint Gap P95 (% diag.) down", "joint_gap_p95_percent_diag"),
        ("Axis Rooted Assets (%) up", "axis_rooted_assets_percentage"),
        ("Axis Support macro (%) up", "axis_support_macro_percentage"),
        ("K=9 Axis Pose Support macro (%) up", "k9_axis_pose_support_macro_percentage"),
        ("Collision-Free Joint Motion Range (%) up", "collision_free_joint_motion_range_macro_percentage"),
        ("Premature Collision-Free Joints (%) up", "premature_collision_free_joints_macro_percentage"),
        ("Penetration Growth P95 (% diag.) down", "penetration_growth_asset_balanced_p95_percent_diag"),
        ("Structural coverage (%)", "structural_coverage_percentage"),
        ("Collision coverage (%)", "collision_coverage_percentage"),
    ]
    csv_lines: list[str] = []
    csv_path = args.out / "table.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=[name for name, _ in columns])
        writer.writeheader()
        for slug, values in summaries.items():
            writer.writerow(
                {
                    name: slug if key is None else values.get(key)
                    for name, key in columns
                }
            )
    header = "| " + " | ".join(name for name, _ in columns) + " |"
    separator = "|" + "|".join("---" if index == 0 else "---:" for index in range(len(columns))) + "|"
    markdown = [header, separator]
    for slug, values in summaries.items():
        markdown.append(
            "| "
            + " | ".join(slug if key is None else _display(values.get(key)) for _, key in columns)
            + " |"
        )
    _atomic_text(args.out / "table.md", "\n".join(markdown) + "\n")
    print(json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
