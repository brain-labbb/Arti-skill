#!/usr/bin/env python3
"""Recompute frozen Nano3D Naming metrics directly from converted N=33 GLBs."""

from __future__ import annotations

import importlib.util
import json
import math
import statistics
from pathlib import Path
from typing import Any

from pygltflib import GLTF2


EXP_ROOT = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
NAMING_SCRIPT = EXP_ROOT / "scripts/run_nano3d_naming.py"
GLB_OUTPUT = EXP_ROOT / "runtime/nano3d_glb_n33/output"
NAMING_OUTPUT = EXP_ROOT / "runtime/nano3d_naming"
GOLD_PATH = EXP_ROOT / "reference/naming_gold_v2.json"
OUTPUT_PATH = EXP_ROOT / "runtime/nano3d_glb_n33/naming_verification.json"
REPORT_PATH = EXP_ROOT / "runtime/nano3d_glb_n33/naming_verification.md"
BASE_FRAME = "__urdf_world__"


def load_naming_module():
    spec = importlib.util.spec_from_file_location("nano3d_naming_frozen", NAMING_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {NAMING_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ratio(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def mean(rows: list[dict[str, Any]], key: str) -> float | None:
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    return statistics.mean(values) if values else None


def micro(rows: list[dict[str, Any]], numerator: str, denominator: str) -> float | None:
    return ratio(
        sum(int(row[numerator]) for row in rows),
        sum(int(row[denominator]) for row in rows),
    )


def glb_names(path: Path) -> tuple[list[str], list[str]]:
    gltf = GLTF2.load_binary(str(path))
    nodes = gltf.nodes or []
    all_names = [node.name or "" for node in nodes if (node.name or "") != BASE_FRAME]
    mesh_names = [node.name or "" for node in nodes if node.mesh is not None]
    if len(all_names) != len(set(all_names)):
        raise RuntimeError(f"duplicate GLB link node names: {path}")
    if len(mesh_names) != len(set(mesh_names)):
        raise RuntimeError(f"duplicate GLB mesh node names: {path}")
    return all_names, mesh_names


def evaluate_glb(
    old: dict[str, Any],
    roles: list[dict[str, Any]],
    path: Path,
    naming: Any,
) -> dict[str, Any]:
    names, mesh_names = glb_names(path)
    assignment = naming.assign_required_roles(mesh_names, roles)
    required_count = len(assignment["required_slots"])
    strong = [row for row in assignment["assignments"] if row["evidence_strength"] == "strong"]
    named = [name for name in names if not naming.PLACEHOLDER_RE.fullmatch(name)]
    named_mesh = [name for name in mesh_names if not naming.PLACEHOLDER_RE.fullmatch(name)]
    repeated = naming.instance_rows(mesh_names, roles)
    functional_required = sum(bool(slot["functional"]) for slot in assignment["required_slots"])
    functional_matched = sum(bool(row["functional"]) for row in assignment["assignments"])
    assigned_names = {row["link"] for row in assignment["assignments"]}
    return {
        "asset_id": old["asset_id"],
        "slug": old["slug"],
        "glb": str(path),
        "semantic_reference_eligible": bool(old["semantic_reference_eligible"]),
        "link_count": len(names),
        "mesh_bearing_link_count": len(mesh_names),
        "named_link_count": len(named),
        "named_mesh_link_count": len(named_mesh),
        "placeholder_link_count": len(names) - len(named),
        "nameability": ratio(len(named_mesh), len(mesh_names)),
        "required_spec_instance_count": required_count,
        "matched_required_instance_count": assignment["matched_required_instances"],
        "source_role_recall": ratio(assignment["matched_required_instances"], required_count),
        "strong_match_count": len(strong),
        "strong_match_sensitivity": ratio(len(strong), required_count),
        "functional_required_instance_count": functional_required,
        "functional_matched_instance_count": functional_matched,
        "functional_core_coverage": ratio(functional_matched, functional_required),
        "paper_aligned_richness_candidate": ratio(len(named_mesh), required_count),
        "instance_rows": repeated,
        "instance_discriminability": ratio(
            sum(row["distinguishable_count"] for row in repeated),
            sum(row["required_count"] for row in repeated),
        ),
        "required_role_assignment": assignment["assignments"],
        "extra_real_part_candidates": sorted(name for name in named_mesh if name not in assigned_names),
    }


def equal_value(left: Any, right: Any) -> bool:
    if isinstance(left, float) or isinstance(right, float):
        if left is None or right is None:
            return left is right
        return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-12)
    return left == right


def assignment_signature(rows: list[dict[str, Any]]) -> list[tuple[Any, ...]]:
    return sorted(
        (
            row.get("slot_id"),
            row.get("role"),
            row.get("link"),
            row.get("score"),
            row.get("evidence_strength"),
        )
        for row in rows
    )


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    eligible = [row for row in rows if row["semantic_reference_eligible"]]
    repeated = [item for row in eligible for item in row["instance_rows"]]
    return {
        "asset_count_direct": len(rows),
        "asset_count_source_semantic": len(eligible),
        "link_count_total": sum(row["link_count"] for row in rows),
        "mesh_bearing_link_count_total": sum(row["mesh_bearing_link_count"] for row in rows),
        "parts_per_asset_mean": mean(rows, "mesh_bearing_link_count"),
        "nameability_micro": micro(rows, "named_mesh_link_count", "mesh_bearing_link_count"),
        "paper_aligned_richness_candidate_mean": mean(eligible, "paper_aligned_richness_candidate"),
        "paper_aligned_richness_candidate_micro": micro(
            eligible, "named_mesh_link_count", "required_spec_instance_count"
        ),
        "source_role_recall_macro": mean(eligible, "source_role_recall"),
        "source_role_recall_micro": micro(
            eligible, "matched_required_instance_count", "required_spec_instance_count"
        ),
        "strong_match_sensitivity_macro": mean(eligible, "strong_match_sensitivity"),
        "strong_match_sensitivity_micro": micro(
            eligible, "strong_match_count", "required_spec_instance_count"
        ),
        "functional_core_coverage_macro": mean(eligible, "functional_core_coverage"),
        "functional_core_coverage_micro": micro(
            eligible, "functional_matched_instance_count", "functional_required_instance_count"
        ),
        "instance_discriminability": ratio(
            sum(item["distinguishable_count"] for item in repeated),
            sum(item["required_count"] for item in repeated),
        ),
        "instance_distinguishable_count": sum(item["distinguishable_count"] for item in repeated),
        "instance_required_count": sum(item["required_count"] for item in repeated),
        "instance_applicable_groups": len(repeated),
    }


def main() -> int:
    naming = load_naming_module()
    gold = json.loads(GOLD_PATH.read_text(encoding="utf-8"))
    old_rows = json.loads((NAMING_OUTPUT / "asset_records.json").read_text(encoding="utf-8"))
    old_summary = json.loads((NAMING_OUTPUT / "summary.json").read_text(encoding="utf-8"))
    glb_rows = []
    comparisons = []
    scalar_keys = [
        "link_count",
        "mesh_bearing_link_count",
        "named_link_count",
        "named_mesh_link_count",
        "placeholder_link_count",
        "nameability",
        "required_spec_instance_count",
        "matched_required_instance_count",
        "source_role_recall",
        "strong_match_count",
        "strong_match_sensitivity",
        "functional_required_instance_count",
        "functional_matched_instance_count",
        "functional_core_coverage",
        "paper_aligned_richness_candidate",
        "instance_discriminability",
    ]
    for old in old_rows:
        roles = gold["assets"][old["slug"]]["roles"]
        path = GLB_OUTPUT / old["asset_id"] / "model.glb"
        current = evaluate_glb(old, roles, path, naming)
        glb_rows.append(current)
        mismatches = [key for key in scalar_keys if not equal_value(current[key], old[key])]
        if assignment_signature(current["required_role_assignment"]) != assignment_signature(old["required_role_assignment"]):
            mismatches.append("required_role_assignment")
        if current["extra_real_part_candidates"] != sorted(old["extra_real_part_candidates"]):
            mismatches.append("extra_real_part_candidates")
        comparisons.append({"asset_id": old["asset_id"], "mismatches": mismatches, "exact": not mismatches})

    glb_summary = summarize(glb_rows)
    summary_keys = list(glb_summary)
    summary_mismatches = [
        key for key in summary_keys if not equal_value(glb_summary[key], old_summary.get(key))
    ]
    result = {
        "protocol": "nano3d_naming_glb_direct_verification_v1",
        "glb_root": str(GLB_OUTPUT),
        "asset_count": len(glb_rows),
        "asset_exact_count": sum(row["exact"] for row in comparisons),
        "all_asset_metrics_exact": all(row["exact"] for row in comparisons),
        "summary_metrics_exact": not summary_mismatches,
        "summary_mismatches": summary_mismatches,
        "glb_summary": glb_summary,
        "glb_asset_records": glb_rows,
        "comparisons": comparisons,
        "scope_note": "N=33 direct/source-semantic metrics only; sibling-seed cross-seed metrics were not recomputed from GLB.",
    }
    OUTPUT_PATH.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    lines = [
        "# Nano3D Naming direct GLB verification",
        "",
        f"资产逐字段一致：{result['asset_exact_count']}/{result['asset_count']}。",
        f"汇总字段一致：{'PASS' if result['summary_metrics_exact'] else 'FAIL'}。",
        "",
        "直接从转换后 GLB 读取全部 link/mesh node names，并使用冻结的 Naming role matcher 与 gold 重算 Parts、Nameability、Recall、Richness、Functional Core Coverage 和 Instance Discriminability。Cross-seed 指标本轮未从 sibling GLB 重算。",
        "",
        "```json",
        json.dumps(glb_summary, indent=2, ensure_ascii=False),
        "```",
    ]
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("asset_count", "asset_exact_count", "all_asset_metrics_exact", "summary_metrics_exact", "summary_mismatches", "glb_summary")}, indent=2, ensure_ascii=False))
    return 0 if result["all_asset_metrics_exact"] and result["summary_metrics_exact"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
