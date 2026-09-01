#!/usr/bin/env python3
"""Score frozen Table 4 Constraints v2 canonical artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXP_ROOT = Path(__file__).resolve().parents[1]
REFERENCE_ROOT = EXP_ROOT / "reference" / "table4_constraints_v2"


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def normalize(value: str) -> str:
    return "_".join(token for token in re.split(r"[^a-z0-9]+", value.lower()) if token)


def alias_match(name: str, alias: str) -> bool:
    normalized_name = f"_{normalize(name)}_"
    normalized_alias = f"_{normalize(alias)}_"
    return normalized_alias in normalized_name


def load_json(path: Path) -> Any:
    return json.loads(contained(path).read_text(encoding="utf-8"))


def load_manifest(path: Path) -> dict[str, dict[str, Any]]:
    rows = {}
    for line_number, line in enumerate(contained(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        task_id = row["task_id"]
        if task_id in rows:
            raise ValueError(f"duplicate task_id at line {line_number}: {task_id}")
        rows[task_id] = row
    return rows


def measure_count(constraint: dict[str, Any], nodes: list[dict[str, Any]]) -> dict[str, Any]:
    aliases = constraint["aliases"]
    by_level: dict[str, list[str]] = {}
    for node in nodes:
        if not node.get("mesh_bearing"):
            continue
        name = str(node.get("name") or "")
        if any(alias_match(name, alias) for alias in aliases):
            by_level.setdefault(str(node.get("level") or "unknown"), []).append(name)
    priority = ("visual", "glb_node", "link")
    selected_level = next((level for level in priority if by_level.get(level)), None)
    matches = sorted(by_level.get(selected_level, [])) if selected_level else []
    measurable = bool(matches)
    observed = len(matches) if measurable else None
    return {
        "measurable": measurable,
        "passed": measurable and observed == constraint["target"],
        "observed": observed,
        "target": constraint["target"],
        "selected_level": selected_level,
        "matches": matches,
        "all_matching_levels": by_level,
    }


def measure_numeric(constraint: dict[str, Any], artifact: dict[str, Any]) -> dict[str, Any]:
    key = f"{constraint['measure']}_m"
    observed = artifact.get(key)
    measurable = isinstance(observed, (int, float)) and math.isfinite(float(observed))
    target = float(constraint["target_m"])
    tolerance = max(
        float(constraint["absolute_tolerance_m"]),
        abs(target) * float(constraint["relative_tolerance"]),
    )
    error = abs(float(observed) - target) if measurable else None
    return {
        "measurable": measurable,
        "passed": measurable and error <= tolerance,
        "observed_m": float(observed) if measurable else None,
        "target_m": target,
        "absolute_error_m": error,
        "tolerance_m": tolerance,
        "measure": constraint["measure"],
    }


def score_task(spec: dict[str, Any], row: dict[str, Any] | None, panel: str) -> dict[str, Any]:
    applicable_types = {
        "structured_main": {"exact_count", "numeric_dimension"},
        "cad_numeric": {"numeric_dimension"},
        "image_count": {"exact_count"},
    }[panel]
    artifact = None
    nodes: list[dict[str, Any]] = []
    errors = []
    if row and row.get("status") == "success":
        canonical_dir = contained(Path(row["canonical_dir"]))
        artifact_path = canonical_dir / "artifact.json"
        nodes_path = canonical_dir / "semantic_nodes.json"
        try:
            artifact = load_json(artifact_path)
            nodes = load_json(nodes_path)
        except Exception as exc:  # fail closed at the item level
            errors.append(f"{type(exc).__name__}: {exc}")
    records = []
    for constraint in spec["constraints"]:
        if constraint["type"] not in applicable_types:
            continue
        base = {
            "task_id": spec["task_id"],
            "constraint_id": constraint["constraint_id"],
            "type": constraint["type"],
        }
        if artifact is None:
            measurement = {"measurable": False, "passed": False, "reason": "missing_or_invalid_artifact"}
        elif constraint["type"] == "exact_count":
            measurement = measure_count(constraint, nodes)
        else:
            measurement = measure_numeric(constraint, artifact)
        records.append({**base, **measurement})
    return {
        "task_id": spec["task_id"],
        "category": spec["category"],
        "artifact_registered": artifact is not None,
        "errors": errors,
        "constraints": records,
    }


def ratio(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def display_ratio(value: float | None) -> str:
    return f"{value:.3f}" if value is not None else "N/A"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", required=True)
    parser.add_argument("--panel", choices=("structured_main", "cad_numeric", "image_count"), required=True)
    parser.add_argument("--artifact-manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    manifest = load_manifest(args.artifact_manifest)
    output = contained(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    protocol = load_json(REFERENCE_ROOT / "protocol.json")
    task_records = []
    for task_id in sorted(protocol["spec_sha256"]):
        spec_path = REFERENCE_ROOT / "specs" / f"{task_id}.json"
        if sha256(spec_path) != protocol["spec_sha256"][task_id]:
            raise RuntimeError(f"frozen spec hash mismatch: {task_id}")
        task_records.append(score_task(load_json(spec_path), manifest.get(task_id), args.panel))
    constraints = [record for task in task_records for record in task["constraints"]]
    total = len(constraints)
    measurable = sum(bool(row["measurable"]) for row in constraints)
    passed = sum(bool(row["passed"]) for row in constraints)
    count_rows = [row for row in constraints if row["type"] == "exact_count"]
    numeric_rows = [row for row in constraints if row["type"] == "numeric_dimension"]
    summary = {
        "schema_version": 2,
        "benchmark_id": protocol["benchmark_id"],
        "prompt_manifest_sha256": protocol["prompt_manifest_sha256"],
        "method": args.method,
        "panel": args.panel,
        "task_count": len(task_records),
        "artifact_count": sum(task["artifact_registered"] for task in task_records),
        "constraints": total,
        "measurable": measurable,
        "passed": passed,
        "coverage": ratio(measurable, total),
        "satisfaction": ratio(passed, total),
        "conditional_accuracy": ratio(passed, measurable),
        "count_pass": f"{sum(row['passed'] for row in count_rows)}/{len(count_rows)}" if count_rows else None,
        "numeric_pass": f"{sum(row['passed'] for row in numeric_rows)}/{len(numeric_rows)}" if numeric_rows else None,
    }
    (output / "records.json").write_text(
        json.dumps(task_records, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    report = [
        f"# Table 4 Constraints v2: {args.method}", "",
        f"Panel: `{args.panel}`", "",
        "| Artifacts | Constraints | Measurable | Passed | Coverage | Satisfaction | Conditional | Count Pass | Numeric Pass |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| {summary['artifact_count']}/{summary['task_count']} | {total} | {measurable} | {passed} | "
        f"{display_ratio(summary['coverage'])} | {display_ratio(summary['satisfaction'])} | "
        f"{display_ratio(summary['conditional_accuracy'])} | {summary['count_pass'] or 'N/A'} | "
        f"{summary['numeric_pass'] or 'N/A'} |",
        "",
        f"Frozen prompt manifest SHA-256: `{protocol['prompt_manifest_sha256']}`.", "",
    ]
    (output / "report.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
