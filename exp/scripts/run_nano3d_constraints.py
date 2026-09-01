#!/usr/bin/env python3
"""Evaluate frozen source-derived Nano3D constraints over 33 x 36 seed packages.

All inputs and outputs are under exp.  The harness deliberately separates
direct/source-derived constraints from unsupported hidden-spec claims.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any


EXP = Path("/mnt/zsn/lyb/arti-skill/exp").resolve()
SCRIPT_DIR = EXP / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_nano3d_naming import assign_required_roles, parse_links, pattern_match, tokens  # noqa: E402


GOLD = EXP / "reference/naming_gold_v2.json"
PROTOCOL = EXP / "reference/constraints_protocol_v1.json"
SOURCE_ROOT = EXP / "reference/naming_sources"
RELIABILITY = EXP / "runtime/nano3d_seed_reliability"
DEFAULT_OUTPUT = EXP / "runtime/nano3d_constraints_v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_vector(text: str | None, default: tuple[float, float, float] | None = None) -> tuple[float, ...] | None:
    if text is None:
        return default
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError:
        return None
    return values if len(values) == 3 and all(math.isfinite(value) for value in values) else None


def norm(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values))


def mesh_path(package: Path, filename: str) -> Path | None:
    raw = filename.strip()
    if not raw:
        return None
    if raw.startswith("package://"):
        raw = raw.removeprefix("package://")
    if raw.startswith("file://"):
        raw = raw.removeprefix("file://")
    path = Path(raw)
    return path if path.is_absolute() else (package / path).resolve()


def relation_patterns(role: str, role_specs: list[dict[str, Any]]) -> list[str]:
    for spec in role_specs:
        if str(spec["name"]) == role:
            return [str(value) for value in spec.get("patterns", [])] or [role]
    return [role]


def matching_links(names: list[str], role: str, role_specs: list[dict[str, Any]]) -> set[str]:
    patterns = relation_patterns(role, role_specs)
    return {name for name in names if any(pattern_match(pattern, name) for pattern in patterns)}


def applicable_design_requirements(
    slug: str, evidence: str, config: dict[str, Any]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], str | None]:
    if not evidence.startswith("designs/") or "+" in evidence:
        return [], [], None
    path = (SOURCE_ROOT / evidence).resolve()
    if not path.is_file():
        return [], [], None
    design = json.loads(path.read_text(encoding="utf-8"))
    role_requirements: dict[str, dict[str, Any]] = {}
    rows: list[dict[str, Any]] = []
    for anchor in design.get("category_anchors", []):
        applies = anchor.get("applies_to") or {}
        slot = applies.get("slot")
        candidates = applies.get("candidates") or []
        if slot and candidates and config.get(slot) not in candidates:
            continue
        for requirement in anchor.get("required_roles") or []:
            role = str(requirement.get("role", ""))
            if not role:
                continue
            if requirement.get("exact_count") is not None:
                expected = int(requirement["exact_count"])
                source = "exact_count"
            elif "count_from" in requirement and requirement["count_from"] in config:
                expected = int(config[requirement["count_from"]])
                source = f"count_from:{requirement['count_from']}"
            else:
                continue
            previous = role_requirements.get(role)
            if previous is None or expected > previous["minimum"]:
                role_requirements[role] = {
                    "constraint_id": f"{slug}:count:{role}", "role": role,
                    "minimum": expected, "source": source,
                }
        for index, relation in enumerate(anchor.get("required_relations") or []):
            rows.append({
                "constraint_id": f"{slug}:relation:{anchor.get('anchor_id', 'anchor')}:{index}",
                "anchor_id": anchor.get("anchor_id"),
                **relation,
            })
    return sorted(role_requirements.values(), key=lambda row: row["role"]), rows, str(path)


def evaluate_seed(slug: str, spec: dict[str, Any], outcome: dict[str, Any]) -> dict[str, Any]:
    seed = int(outcome["seed"])
    package = Path(outcome["artifact_dir"]).resolve()
    urdf = package / "model.urdf"
    config = outcome.get("config") or outcome.get("qc", {}).get("config") or {}
    qc = outcome.get("qc") or {}
    root = ET.parse(urdf).getroot()
    link_nodes = root.findall("link")
    joint_nodes = root.findall("joint")
    link_names = [node.attrib.get("name", "") for node in link_nodes if node.attrib.get("name")]
    link_set = set(link_names)
    visual_names = [row["name"] for row in parse_links(urdf) if row["has_visual"]]
    clauses: list[dict[str, Any]] = []

    count_requirements, relations, relation_source = applicable_design_requirements(
        slug, str(spec.get("evidence", "")), config
    )
    count_roles = []
    for requirement in count_requirements:
        patterns = relation_patterns(requirement["role"], spec["roles"])
        count_roles.append({"name": requirement["role"], "patterns": patterns, "min_count": requirement["minimum"]})
    assignment = assign_required_roles(visual_names, count_roles) if count_roles else {"assignments": []}
    assigned_counts: defaultdict[str, int] = defaultdict(int)
    for row in assignment["assignments"]:
        assigned_counts[str(row["role"])] += 1
    for requirement in count_requirements:
        role = requirement["role"]
        required = requirement["minimum"]
        observed = assigned_counts[role]
        clauses.append({
            "constraint_id": requirement["constraint_id"],
            "category": "count", "measurable": True, "passed": observed >= required,
            "expected": {"minimum": required}, "observed": observed,
            "evidence": relation_source, "requirement_source": requirement["source"],
        })

    edges = []
    child_names = set()
    numeric_index = 0
    for joint in joint_nodes:
        name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "")
        parent_node, child_node = joint.find("parent"), joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        child_names.add(child)
        axis_node = joint.find("axis")
        axis = finite_vector(axis_node.attrib.get("xyz") if axis_node is not None else None, (1.0, 0.0, 0.0))
        edges.append({"name": name, "type": joint_type, "parent": parent, "child": child, "axis": axis})
        if joint_type == "fixed":
            continue
        numeric_index += 1
        axis_valid = axis is not None and norm(axis) > 1e-9
        limits_valid = True
        observed_limits = None
        if joint_type in {"revolute", "prismatic"}:
            limit = joint.find("limit")
            try:
                lower = float(limit.attrib["lower"]) if limit is not None else math.nan
                upper = float(limit.attrib["upper"]) if limit is not None else math.nan
                limits_valid = math.isfinite(lower) and math.isfinite(upper) and lower <= upper
                observed_limits = [lower, upper]
            except (KeyError, ValueError):
                limits_valid = False
        clauses.append({
            "constraint_id": f"{slug}:numeric:{name or numeric_index}",
            "category": "numeric", "measurable": True, "passed": axis_valid and limits_valid,
            "expected": "finite non-zero axis and finite ordered limits when bounded",
            "observed": {"axis": axis, "limits": observed_limits, "joint_type": joint_type},
        })

    for relation in relations:
        parents = matching_links(link_names, str(relation.get("parent_role", "")), spec["roles"])
        children = matching_links(link_names, str(relation.get("child_role", "")), spec["roles"])
        expected_type = relation.get("joint_type")
        expected_axis_raw = relation.get("axis")
        expected_axis = tuple(float(value) for value in expected_axis_raw) if expected_axis_raw else None
        candidates = [edge for edge in edges if edge["parent"] in parents and edge["child"] in children]
        typed = [edge for edge in candidates if not expected_type or edge["type"] == expected_type]
        if expected_axis is not None:
            axis_matches = []
            for edge in typed:
                actual = edge["axis"]
                cosine = (
                    sum(a * b for a, b in zip(actual, expected_axis)) / (norm(actual) * norm(expected_axis))
                    if actual is not None and norm(actual) > 0 and norm(expected_axis) > 0 else -1.0
                )
                if cosine >= 0.999:
                    axis_matches.append(edge)
            typed = axis_matches
        clauses.append({
            "constraint_id": relation["constraint_id"], "category": "relational",
            "measurable": True, "passed": bool(typed),
            "expected": {key: relation.get(key) for key in ("parent_role", "child_role", "joint_type", "axis")},
            "observed": {"parent_candidates": sorted(parents), "child_candidates": sorted(children), "matching_edges": typed},
            "evidence": relation_source,
        })

    roots = link_set - child_names
    valid_edges = all(edge["parent"] in link_set and edge["child"] in link_set for edge in edges)
    mesh_refs = [node.attrib.get("filename", "") for node in root.findall(".//mesh")]
    missing_meshes = [name for name in mesh_refs if (path := mesh_path(package, name)) is None or not path.exists()]
    connected_tree = len(roots) == 1 and len(edges) == max(0, len(link_set) - 1) and valid_edges
    strict_qc = qc.get("verdict") == "pass"
    clauses.append({
        "constraint_id": f"{slug}:interface:seed", "category": "interface",
        "measurable": True, "passed": connected_tree and not missing_meshes and strict_qc,
        "expected": "connected URDF, resolved meshes, strict policy pass",
        "observed": {"roots": sorted(roots), "edges": len(edges), "links": len(link_set), "missing_meshes": missing_meshes, "strict_qc": strict_qc},
    })

    motion = qc.get("motion_qc_report") or {}
    joint_edge_pass = all(set(row.get("required_edges", [])) <= set(row.get("covered_edges", [])) for row in motion.get("joints", []))
    motion_pass = (
        motion.get("collision_free") is True
        and not motion.get("missing_edges")
        and not motion.get("missing_pose_ids")
        and joint_edge_pass
    )
    clauses.append({
        "constraint_id": f"{slug}:kinematic:seed", "category": "kinematic",
        "measurable": bool(motion), "passed": motion_pass if motion else None,
        "expected": "collision-free required motion-QC coverage",
        "observed": {"collision_free": motion.get("collision_free"), "missing_edges": motion.get("missing_edges"), "missing_pose_ids": motion.get("missing_pose_ids"), "joint_edge_pass": joint_edge_pass},
    })
    clauses.append({
        "constraint_id": f"{slug}:compatibility:valid-seed", "category": "compatibility",
        "measurable": True, "passed": strict_qc,
        "expected": "resolved valid combination accepted by strict Full-QC",
        "observed": {"config": config, "qc_verdict": qc.get("verdict")},
    })

    measurable = [row for row in clauses if row["measurable"]]
    return {
        "asset_id": f"{slug}__seed_{seed}", "slug": slug, "seed": seed,
        "artifact_dir": str(package), "urdf_sha256": sha256(urdf),
        "clauses": clauses, "measurable": len(measurable),
        "passed": sum(row["passed"] is True for row in measurable),
        "all_pass": bool(measurable) and all(row["passed"] is True for row in measurable),
    }


def aggregate(records: list[dict[str, Any]]) -> dict[str, Any]:
    categories: dict[str, dict[str, int]] = {}
    all_clauses = [clause for record in records for clause in record["clauses"]]
    for category in ("count", "numeric", "relational", "interface", "kinematic", "compatibility"):
        rows = [row for row in all_clauses if row["category"] == category]
        categories[category] = {
            "total": len(rows), "measurable": sum(row["measurable"] for row in rows),
            "passed": sum(row["measurable"] and row["passed"] is True for row in rows),
        }
    total = len(all_clauses)
    measurable = sum(row["measurable"] for row in all_clauses)
    passed = sum(row["measurable"] and row["passed"] is True for row in all_clauses)
    by_template = []
    for slug in sorted({row["slug"] for row in records}):
        seeds = [row for row in records if row["slug"] == slug]
        by_template.append({
            "slug": slug, "seed_count": len(seeds),
            "all_pass_seed_count": sum(row["all_pass"] for row in seeds),
            "all_36_pass": len(seeds) == 36 and all(row["all_pass"] for row in seeds),
        })
    return {
        "schema_version": 1, "protocol": "nano3d_source_derived_constraints_v1",
        "seed_asset_count": len(records), "template_count": len(by_template),
        "total_constraints": total, "measurable_constraints": measurable,
        "passed_constraints": passed,
        "coverage": measurable / total if total else None,
        "satisfaction": passed / total if total else None,
        "conditional_accuracy": passed / measurable if measurable else None,
        "categories": categories,
        "all_pass_assets": sum(row["all_pass"] for row in records),
        "all_pass_templates_36_of_36": sum(row["all_36_pass"] for row in by_template),
        "templates": by_template,
        "invalid_combination_rejection": None,
        "invalid_combination_status": "unsupported_without_frozen_negative_manifest",
    }


def write_report(summary: dict[str, Any], path: Path) -> None:
    def ratio(row: dict[str, int]) -> str:
        return f"{row['passed']}/{row['measurable']} = {row['passed']/row['measurable']:.3f}" if row["measurable"] else "N/A"
    categories = summary["categories"]
    lines = [
        "# Nano3D source-derived Constraints v1", "",
        "本结果评测 33 个既有模板 × seeds 0–35 的 1,188 个最终 URDF package。它是本地 source-derived/operational constraint audit，不是论文 52 条 hidden-spec GLB constraint 的复现。", "",
        "| Metric | Result |", "|---|---:|",
        f"| Coverage | {summary['measurable_constraints']}/{summary['total_constraints']} = {summary['coverage']:.3f} |",
        f"| Satisfaction | {summary['passed_constraints']}/{summary['total_constraints']} = {summary['satisfaction']:.3f} |",
        f"| Conditional Accuracy | {summary['passed_constraints']}/{summary['measurable_constraints']} = {summary['conditional_accuracy']:.3f} |",
        f"| Count Pass | {ratio(categories['count'])} |",
        f"| Numeric Pass | {ratio(categories['numeric'])} |",
        f"| Relational Pass | {ratio(categories['relational'])} |",
        f"| Interface Pass | {ratio(categories['interface'])} |",
        f"| Kinematic Pass | {ratio(categories['kinematic'])} |",
        f"| Compatibility Pass (valid configs) | {ratio(categories['compatibility'])} |",
        f"| All-Pass Seed Assets | {summary['all_pass_assets']}/{summary['seed_asset_count']} = {summary['all_pass_assets']/summary['seed_asset_count']:.3f} |",
        f"| 36/36 All-Pass Templates | {summary['all_pass_templates_36_of_36']}/{summary['template_count']} |",
        "| Invalid Combination Rejection | N/A（无冻结 negative manifest） |", "",
        "限制：count 是 required-role lower-bound；numeric 是 URDF joint 数值有效性；interface/compatibility 是 operational proxy；详见 `constraints_protocol_v1.json`。", "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output.resolve()
    if EXP not in output.parents:
        raise RuntimeError(f"output must be inside {EXP}: {output}")
    output.mkdir(parents=True, exist_ok=True)
    gold = json.loads(GOLD.read_text(encoding="utf-8"))
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    records = []
    for slug, spec in sorted(gold["assets"].items()):
        outcomes_path = RELIABILITY / "templates" / slug / "outcomes.json"
        payload = json.loads(outcomes_path.read_text(encoding="utf-8"))
        if len(payload.get("outcomes", [])) != 36:
            raise RuntimeError(f"{slug}: expected 36 outcomes")
        records.extend(evaluate_seed(slug, spec, row) for row in payload["outcomes"])
    summary = aggregate(records)
    summary["input_hashes"] = {"naming_gold_v2.json": sha256(GOLD), "constraints_protocol_v1.json": sha256(PROTOCOL)}
    summary["limitations"] = protocol["limitations"]
    (output / "records.json").write_text(json.dumps(records, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_report(summary, output / "report.md")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
