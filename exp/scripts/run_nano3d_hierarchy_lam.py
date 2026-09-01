#!/usr/bin/env python3
"""Run the shared Nano3D hierarchy metrics on provenance-checked LAM URDFs.

This adapter is deliberately fail-closed.  It never discovers URDFs by name:
the caller must provide an explicit JSONL manifest whose records identify LAM
as the producing method and point to final URDF artifacts.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any

from run_nano3d_hierarchy import parse_hierarchy, sha256


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = EXP_ROOT / "reference/hierarchy_baselines/lam_manifest.jsonl"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy_baselines/lam"
METHOD = "LAM"
PROTOCOL = "nano3d_hierarchy_lam_shared_tree_v1"


def contained(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def load_manifest(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"line {line_number}: expected a JSON object")
        rows.append(value)
    if not rows:
        raise ValueError("manifest contains no asset records")
    return rows


def validate_row(row: dict[str, Any], index: int, workspace: Path) -> tuple[Path | None, list[str]]:
    errors: list[str] = []
    asset_id = row.get("asset_id")
    if not isinstance(asset_id, str) or not asset_id.strip():
        errors.append(f"record {index}: missing non-empty asset_id")
    if str(row.get("method", "")).strip().casefold() != METHOD.casefold():
        errors.append(f"record {index}: method must be exactly {METHOD!r}")
    provenance = row.get("provenance")
    if not isinstance(provenance, dict) or not any(
        str(provenance.get(key, "")).strip()
        for key in ("run_id", "source_repository", "generation_record")
    ):
        errors.append(
            f"record {index}: provenance must include run_id, source_repository, or generation_record"
        )

    raw_urdf = row.get("final_urdf")
    if not isinstance(raw_urdf, str) or not raw_urdf.strip():
        errors.append(f"record {index}: missing final_urdf")
        return None, errors

    candidate = Path(raw_urdf)
    if not candidate.is_absolute():
        candidate = workspace / candidate
    lexical = Path(candidate.absolute())
    if not contained(lexical, workspace):
        errors.append(f"record {index}: final_urdf is outside workspace: {lexical}")
        return None, errors
    if not lexical.exists():
        errors.append(f"record {index}: final_urdf does not exist: {lexical}")
        return None, errors
    resolved = lexical.resolve()
    if not contained(resolved, workspace):
        errors.append(f"record {index}: final_urdf resolves outside workspace: {resolved}")
        return None, errors
    if not resolved.is_file() or resolved.suffix.casefold() != ".urdf":
        errors.append(f"record {index}: final_urdf must be a .urdf file: {resolved}")
        return None, errors
    return resolved, errors


def blocked_summary(manifest: Path, reasons: list[str]) -> dict[str, Any]:
    return {
        "protocol": PROTOCOL,
        "method": METHOD,
        "status": "BLOCKED",
        "validated": False,
        "manifest": str(manifest),
        "evaluation_unit": "one final LAM-produced articulated URDF per prompt",
        "asset_count": 0,
        "metrics": None,
        "table_3_result": {
            "valid_tree": "N/R",
            "has_hierarchy": "N/R",
            "semantic_depth": "N/R",
            "named_groups": "N/R",
            "pivots": "N/R",
            "parent_child_edge_f1": "N/R",
            "hierarchy_exact_match": "N/R",
            "semantic_nesting_accuracy": "N/R",
            "cross_seed_consistency": "N/A (LAM is a per-asset method)",
        },
        "blocking_reasons": reasons,
        "required_to_run": [
            "a non-empty explicit JSONL manifest",
            "one locally present final_urdf per record",
            "method=\"LAM\" and auditable generation provenance per record",
        ],
        "not_substituted": [
            "paper-reported numbers",
            "URDFs produced by other methods",
            "filename-only LAM guesses",
        ],
    }


def render_report(summary: dict[str, Any]) -> str:
    lines = [
        "# LAM Table 3 Hierarchy Rerun",
        "",
        f"- Status: **{summary['status']}**",
        f"- Validated: `{str(summary['validated']).lower()}`",
        f"- Protocol: `{summary['protocol']}`",
        f"- Manifest: `{summary['manifest']}`",
        f"- N: `{summary['asset_count']}`",
        "",
    ]
    if summary["status"] == "BLOCKED":
        lines.extend(["## Blocking reasons", ""])
        lines.extend(f"- {reason}" for reason in summary["blocking_reasons"])
        lines.extend(
            [
                "",
                "No hierarchy metric was computed. Paper values and artifacts from other methods were not substituted.",
                "",
                "## Required inputs",
                "",
            ]
        )
        lines.extend(f"- {item}" for item in summary["required_to_run"])
        lines.extend(
            [
                "",
                "## Table 3 disposition",
                "",
                "All shared-tree metric cells remain `N/R`; Cross-Seed Consistency is `N/A` because LAM is evaluated per asset.",
            ]
        )
    else:
        metrics = summary["metrics"]
        lines.extend(
            [
                "## Shared-tree results",
                "",
                f"- Valid Tree: {metrics['valid_tree_count']}/{summary['asset_count']} = {metrics['valid_tree_rate']:.6f}",
                f"- Has Hierarchy: {metrics['has_hierarchy_count']}/{summary['asset_count']} = {metrics['has_hierarchy_rate']:.6f}",
                f"- Semantic Depth: {metrics['semantic_depth_mean']:.6f} mean [{metrics['semantic_depth_min']}, {metrics['semantic_depth_max']}]",
                f"- Named Groups: {metrics['named_groups_mean']:.6f} mean; {metrics['named_groups_total']} total",
                f"- Pivots: {metrics['pivots_mean']:.6f} mean; {metrics['pivots_total']} total",
                "- Parent-Child Edge F1: N/A (no independent hierarchy gold)",
                "- Hierarchy Exact Match: N/A (no independent hierarchy gold)",
                "- Semantic Nesting Accuracy: N/A (no independent hierarchy gold)",
                "- Cross-Seed Consistency: N/A (LAM is evaluated per asset)",
            ]
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workspace-root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    workspace = args.workspace_root.resolve(strict=True)
    output = args.output.resolve(strict=False)
    if not contained(output, workspace):
        parser.error(f"output must be inside workspace root: {workspace}")
    output.mkdir(parents=True, exist_ok=True)

    manifest = args.manifest.resolve(strict=False)
    reasons: list[str] = []
    if not contained(manifest, workspace):
        reasons.append(f"manifest path is outside workspace: {manifest}")
    elif not manifest.exists():
        reasons.append(f"LAM final-asset manifest is missing: {manifest}")

    rows: list[dict[str, Any]] = []
    if not reasons:
        try:
            rows = load_manifest(manifest)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            reasons.append(f"manifest is not usable: {type(exc).__name__}: {exc}")

    checked: list[tuple[dict[str, Any], Path]] = []
    asset_ids: set[str] = set()
    for index, row in enumerate(rows, 1):
        path, row_errors = validate_row(row, index, workspace)
        reasons.extend(row_errors)
        asset_id = str(row.get("asset_id", ""))
        if asset_id in asset_ids:
            reasons.append(f"record {index}: duplicate asset_id: {asset_id}")
        asset_ids.add(asset_id)
        if path is not None and not row_errors:
            checked.append((row, path))

    if reasons:
        summary = blocked_summary(manifest, reasons)
        (output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        (output / "report.md").write_text(render_report(summary), encoding="utf-8")
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return 2

    records: list[dict[str, Any]] = []
    parse_errors: list[str] = []
    for row, urdf in checked:
        try:
            metrics = parse_hierarchy(urdf)
        except Exception as exc:  # noqa: BLE001
            parse_errors.append(f"{row['asset_id']}: {type(exc).__name__}: {exc}")
            continue
        records.append(
            {
                "asset_id": row["asset_id"],
                "method": METHOD,
                "prompt_id": row.get("prompt_id"),
                "final_urdf": str(urdf),
                "final_urdf_sha256": sha256(urdf),
                "provenance": row["provenance"],
                **metrics,
            }
        )

    if parse_errors:
        summary = blocked_summary(manifest, ["shared evaluator parse failure: " + item for item in parse_errors])
    else:
        count = len(records)
        summary = {
            "protocol": PROTOCOL,
            "shared_evaluator": "run_nano3d_hierarchy.parse_hierarchy",
            "method": METHOD,
            "status": "VALIDATED",
            "validated": True,
            "manifest": str(manifest),
            "evaluation_unit": "one final LAM-produced articulated URDF per prompt",
            "asset_count": count,
            "metrics": {
                "valid_tree_count": sum(bool(row["valid_tree"]) for row in records),
                "valid_tree_rate": mean(bool(row["valid_tree"]) for row in records),
                "has_hierarchy_count": sum(bool(row["has_tree"]) for row in records),
                "has_hierarchy_rate": mean(bool(row["has_tree"]) for row in records),
                "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in records),
                "semantic_depth_min": min(int(row["semantic_depth"]) for row in records),
                "semantic_depth_max": max(int(row["semantic_depth"]) for row in records),
                "named_groups_mean": mean(int(row["named_group_count"]) for row in records),
                "named_groups_total": sum(int(row["named_group_count"]) for row in records),
                "pivots_mean": mean(int(row["pivot_count"]) for row in records),
                "pivots_total": sum(int(row["pivot_count"]) for row in records),
                "parent_child_edge_f1": None,
                "hierarchy_exact_match": None,
                "semantic_nesting_accuracy": None,
                "cross_seed_consistency": None,
            },
            "metric_notes": {
                "correctness_metrics": "N/A: no independent hierarchy gold",
                "cross_seed_consistency": "N/A: LAM is a per-asset method",
            },
        }

    (output / "asset_records.json").write_text(
        json.dumps(records, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (output / "report.md").write_text(render_report(summary), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["validated"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
