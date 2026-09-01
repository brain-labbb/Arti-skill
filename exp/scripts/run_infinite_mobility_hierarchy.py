#!/usr/bin/env python3
"""Evaluate Table 3 hierarchy metrics on the frozen Infinite Mobility cohort.

This script never invokes Blender. It consumes the terminal baseline records and
independently re-parses every generated URDF using the same conservative
URDF-equivalent mapping as ``run_nano3d_hierarchy.py``.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import hashlib
import itertools
import json
from pathlib import Path
from statistics import mean
from typing import Any
import xml.etree.ElementTree as ET


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROTOCOL = EXP_ROOT / "reference/infinite_mobility_protocol_v1.json"
DEFAULT_INPUT = EXP_ROOT / "runtime/infinite_mobility_v1"
DEFAULT_OUTPUT = (
    EXP_ROOT / "runtime/nano3d_hierarchy_baselines/infinite_mobility"
)
MOVABLE_TYPES = {"revolute", "continuous", "prismatic", "planar", "floating"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def ensure_contained(path: Path, root: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    resolved.relative_to(root.resolve(strict=True))
    return resolved


def mode_rate(values: list[object]) -> float | None:
    if not values:
        return None
    return Counter(values).most_common(1)[0][1] / len(values)


def pairwise_exact_rate(values: list[object]) -> float | None:
    comparisons = [left == right for left, right in itertools.combinations(values, 2)]
    return mean(comparisons) if comparisons else None


def canonical_shape(
    root: str,
    adjacency: dict[str, list[tuple[str, str]]],
    visual_links: set[str],
) -> str:
    children = sorted(
        f"{joint_type}:{canonical_shape(child, adjacency, visual_links)}"
        for child, joint_type in adjacency.get(root, [])
    )
    node_kind = "visual" if root in visual_links else "group"
    return f"{node_kind}[{','.join(children)}]"


def parse_hierarchy(urdf_path: Path, package_root: Path) -> dict[str, Any]:
    xml_root = ET.parse(urdf_path).getroot()
    if xml_root.tag != "robot":
        raise ValueError(f"expected robot root, found {xml_root.tag!r}")
    link_nodes = xml_root.findall("link")
    joint_nodes = xml_root.findall("joint")
    links = [node.attrib.get("name", "") for node in link_nodes]
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        raise ValueError("links are empty, unnamed, or duplicated")
    link_set = set(links)
    visual_links = {
        node.attrib.get("name", "")
        for node in link_nodes
        if node.find("visual/geometry") is not None
    }

    adjacency: dict[str, list[str]] = {name: [] for name in link_set}
    typed_adjacency: dict[str, list[tuple[str, str]]] = {
        name: [] for name in link_set
    }
    children: set[str] = set()
    indegrees: Counter[str] = Counter()
    edge_records: list[tuple[str, str, str, str]] = []
    malformed_edges: list[str] = []
    movable_joint_names: list[str] = []
    for joint in joint_nodes:
        name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "")
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in link_set or child not in link_set:
            malformed_edges.append(name or f"{parent}->{child}")
            continue
        adjacency[parent].append(child)
        typed_adjacency[parent].append((child, joint_type))
        children.add(child)
        indegrees[child] += 1
        edge_records.append((parent, name, joint_type, child))
        if joint_type in MOVABLE_TYPES:
            movable_joint_names.append(name)

    roots = sorted(link_set - children)
    visited: set[str] = set()
    active: set[str] = set()
    cyclic = False

    def visit(node: str) -> None:
        nonlocal cyclic
        if node in active:
            cyclic = True
            return
        if node in visited:
            return
        active.add(node)
        for child in adjacency.get(node, []):
            visit(child)
        active.remove(node)
        visited.add(node)

    for link in link_set:
        visit(link)

    reachable: set[str] = set()
    semantic_depth = 0
    queue: deque[tuple[str, int]] = deque((root, 1) for root in roots)
    while queue:
        node, depth = queue.popleft()
        if node in reachable:
            continue
        reachable.add(node)
        semantic_depth = max(semantic_depth, depth)
        queue.extend((child, depth + 1) for child in adjacency.get(node, []))

    valid_tree = (
        len(roots) == 1
        and not cyclic
        and reachable == link_set
        and all(indegrees[name] <= 1 for name in link_set)
        and len(edge_records) == len(link_set) - 1
        and not malformed_edges
    )
    named_groups = sorted(
        name for name in link_set if adjacency.get(name) and name not in visual_links
    )

    mesh_paths: list[str] = []
    for mesh in xml_root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename or Path(filename).is_absolute():
            raise ValueError(f"non-portable mesh path: {filename!r}")
        mesh_path = (urdf_path.parent / filename).resolve(strict=True)
        mesh_path.relative_to(package_root)
        if not mesh_path.is_file() or mesh_path.stat().st_size == 0:
            raise ValueError(f"missing or empty mesh: {filename!r}")
        mesh_paths.append(mesh_path.relative_to(package_root).as_posix())
    if not mesh_paths:
        raise ValueError("URDF has no mesh references")

    return {
        "valid_tree": valid_tree,
        "root_names": roots,
        "link_count": len(link_set),
        "joint_count": len(joint_nodes),
        "edge_count": len(edge_records),
        "has_hierarchy": bool(valid_tree and semantic_depth > 1),
        "semantic_depth": semantic_depth,
        "named_group_count": len(named_groups),
        "named_group_names": named_groups,
        "pivot_count": len(movable_joint_names),
        "pivot_names": sorted(movable_joint_names),
        "visual_link_count": len(visual_links),
        "mesh_reference_count": len(mesh_paths),
        "edge_signature": [list(edge) for edge in sorted(edge_records)],
        "canonical_shape_signature": (
            canonical_shape(roots[0], typed_adjacency, visual_links)
            if valid_tree else None
        ),
        "malformed_edges": malformed_edges,
    }


def evaluate_case(
    input_root: Path,
    record: dict[str, Any],
) -> dict[str, Any]:
    factory = str(record["factory"])
    seed = int(record["seed"])
    case_dir = ensure_contained(
        input_root / "cases" / factory / f"seed_{seed:03d}", input_root
    )
    package_dir = case_dir / "package"
    stdout_path = ensure_contained(case_dir / "stdout.log", input_root)
    stderr_path = ensure_contained(case_dir / "stderr.log", input_root)
    completion_marker = f"NANO3D_CASE_COMPLETE factory={factory} seed={seed}"
    stdout_text = stdout_path.read_text(encoding="utf-8", errors="replace")
    urdfs = sorted(package_dir.rglob("scene.urdf")) if package_dir.is_dir() else []
    result: dict[str, Any] = {
        "factory": factory,
        "seed": seed,
        "baseline_status": record["status"],
        "baseline_elapsed_seconds": record["elapsed_seconds"],
        "case_dir": case_dir.relative_to(input_root).as_posix(),
        "stdout_sha256": sha256(stdout_path),
        "stderr_sha256": sha256(stderr_path),
        "completion_marker_present": completion_marker in stdout_text,
        "recorded_package_sha256": record.get("package_sha256"),
        "urdf_count": len(urdfs),
        "evaluation_status": "MISSING_ASSET",
        "valid_tree": False,
        "has_hierarchy": False,
    }
    if record["status"] != "PASS":
        if urdfs:
            raise ValueError(f"terminal {record['status']} unexpectedly contains URDF")
        result["reason"] = (
            f"baseline terminal {record['status']}; no generated scene.urdf"
        )
        return result

    if record.get("validation", {}).get("strict_pass") is not True:
        raise ValueError("PASS record does not carry strict_pass validation")
    if len(urdfs) != 1:
        raise ValueError(f"PASS case has {len(urdfs)} scene.urdf files")
    urdf_path = ensure_contained(urdfs[0], input_root)
    package_root = ensure_contained(package_dir, input_root)
    metrics = parse_hierarchy(urdf_path, package_root)
    if metrics["valid_tree"] != record["validation"].get("valid_tree"):
        raise ValueError("independent valid-tree result disagrees with baseline record")
    if not result["completion_marker_present"]:
        raise ValueError("PASS case lacks worker completion marker")
    result.update(metrics)
    result.update(
        {
            "evaluation_status": "PASS",
            "urdf_path": urdf_path.relative_to(input_root).as_posix(),
            "urdf_sha256": sha256(urdf_path),
        }
    )
    return result


def aggregate_factory(factory: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    ordered = sorted(rows, key=lambda item: int(item["seed"]))
    valid = [row for row in ordered if row["evaluation_status"] == "PASS"]
    raw_signatures = [
        tuple(tuple(edge) for edge in row["edge_signature"]) for row in valid
    ]
    canonical_signatures = [row["canonical_shape_signature"] for row in valid]
    depths = [int(row["semantic_depth"]) for row in valid]
    groups = [int(row["named_group_count"]) for row in valid]
    pivots = [int(row["pivot_count"]) for row in valid]
    return {
        "factory": factory,
        "requested_seed_count": len(ordered),
        "valid_asset_count": len(valid),
        "missing_asset_count": len(ordered) - len(valid),
        "all_36_assets_present": len(valid) == 36,
        "valid_tree_count": sum(bool(row["valid_tree"]) for row in ordered),
        "has_hierarchy_count": sum(bool(row["has_hierarchy"]) for row in ordered),
        "semantic_depth_mean": mean(depths) if depths else None,
        "named_groups_mean": mean(groups) if groups else None,
        "pivots_mean": mean(pivots) if pivots else None,
        "raw_edge_signature_mode_rate": mode_rate(raw_signatures),
        "pairwise_raw_edge_exact_rate": pairwise_exact_rate(raw_signatures),
        "canonical_shape_signature_mode_rate": mode_rate(canonical_signatures),
        "pairwise_canonical_shape_exact_rate": pairwise_exact_rate(
            canonical_signatures
        ),
        "semantic_depth_mode_rate": mode_rate(depths),
        "named_group_count_mode_rate": mode_rate(groups),
        "pivot_count_mode_rate": mode_rate(pivots),
        "missing_seeds": [
            int(row["seed"]) for row in ordered if row["evaluation_status"] != "PASS"
        ],
    }


def load_and_validate_inputs(
    protocol_path: Path, input_root: Path
) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    protocol = json.loads(protocol_path.read_text(encoding="utf-8"))
    manifest_path = ensure_contained(input_root / "manifest.json", input_root)
    records_path = ensure_contained(input_root / "records.json", input_root)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    records = json.loads(records_path.read_text(encoding="utf-8"))
    if manifest.get("protocol") != protocol:
        raise ValueError("runtime manifest protocol differs from frozen protocol")
    if manifest.get("factories") != protocol["factories"]:
        raise ValueError("runtime factory selection differs from frozen protocol")
    if manifest.get("seeds") != protocol["seeds"]:
        raise ValueError("runtime seed selection differs from frozen protocol")
    expected = {
        (factory, int(seed))
        for factory in protocol["factories"]
        for seed in protocol["seeds"]
    }
    actual = {(str(row["factory"]), int(row["seed"])) for row in records}
    if actual != expected or len(records) != len(expected):
        raise ValueError(
            f"records do not match frozen Cartesian selection: "
            f"expected={len(expected)} actual={len(records)} unique={len(actual)}"
        )
    terminal = {"PASS", "FAIL", "TIMEOUT", "HARNESS_ERROR"}
    if any(row.get("status") not in terminal for row in records):
        raise ValueError("records contain a non-terminal status")
    return protocol, manifest, records


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--input-root", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    workspace_root = EXP_ROOT.parent.resolve(strict=True)
    protocol_path = ensure_contained(args.protocol, workspace_root)
    input_root = ensure_contained(args.input_root, workspace_root)
    output_root = ensure_contained(
        args.output_root, workspace_root, must_exist=False
    )
    protocol, source_manifest, records = load_and_validate_inputs(
        protocol_path, input_root
    )

    evaluated = [
        evaluate_case(input_root, record)
        for record in sorted(records, key=lambda row: (row["factory"], row["seed"]))
    ]
    factory_rows = [
        aggregate_factory(
            factory,
            [row for row in evaluated if row["factory"] == factory],
        )
        for factory in protocol["factories"]
    ]
    valid = [row for row in evaluated if row["evaluation_status"] == "PASS"]
    valid_trees = [row for row in valid if row["valid_tree"]]
    requested_count = len(evaluated)
    valid_tree_count = sum(bool(row["valid_tree"]) for row in evaluated)
    has_hierarchy_count = sum(bool(row["has_hierarchy"]) for row in evaluated)

    cross_seed_fields = (
        "raw_edge_signature_mode_rate",
        "pairwise_raw_edge_exact_rate",
        "canonical_shape_signature_mode_rate",
        "pairwise_canonical_shape_exact_rate",
        "semantic_depth_mode_rate",
        "named_group_count_mode_rate",
        "pivot_count_mode_rate",
    )
    cross_seed_macro = {
        field: mean(
            float(row[field]) for row in factory_rows if row[field] is not None
        )
        for field in cross_seed_fields
    }
    summary = {
        "protocol_id": "nano3d_infinite_mobility_hierarchy_v1",
        "generated_at": utc_now(),
        "source_protocol_id": protocol["protocol_id"],
        "evaluation_unit": "20 public factories x frozen seeds 0-35",
        "common_category_matched": False,
        "mapping": {
            "scene_node": "named URDF link",
            "flat_depth": 1,
            "semantic_depth": (
                "maximum named-link root-to-leaf path length; root counts as 1"
            ),
            "unnamed_wrapper_collapse": "not applicable: URDF links must be named",
            "named_group": "named link with children and no visual geometry",
            "pivot": "named non-fixed URDF joint",
            "has_hierarchy": "valid tree and semantic_depth > 1",
        },
        "requested_cases": requested_count,
        "generated_asset_count": len(valid),
        "missing_asset_count": requested_count - len(valid),
        "missing_assets": [
            {"factory": row["factory"], "seed": row["seed"], "reason": row["reason"]}
            for row in evaluated if row["evaluation_status"] != "PASS"
        ],
        "worker_completion_marker_count": sum(
            bool(row["completion_marker_present"]) for row in evaluated
        ),
        "independently_validated_mesh_reference_total": sum(
            int(row.get("mesh_reference_count", 0)) for row in evaluated
        ),
        "table3": {
            "valid_tree_count": valid_tree_count,
            "valid_tree_denominator_requested": requested_count,
            "valid_tree_rate_requested": valid_tree_count / requested_count,
            "valid_tree_denominator_generated": len(valid),
            "valid_tree_rate_generated": valid_tree_count / len(valid),
            "has_hierarchy_count": has_hierarchy_count,
            "has_hierarchy_denominator_requested": requested_count,
            "has_hierarchy_rate_requested": has_hierarchy_count / requested_count,
            "has_hierarchy_denominator_generated": len(valid),
            "has_hierarchy_rate_generated": has_hierarchy_count / len(valid),
            "semantic_depth_mean": mean(
                int(row["semantic_depth"]) for row in valid_trees
            ),
            "semantic_depth_min": min(
                int(row["semantic_depth"]) for row in valid_trees
            ),
            "semantic_depth_max": max(
                int(row["semantic_depth"]) for row in valid_trees
            ),
            "named_groups_mean": mean(
                int(row["named_group_count"]) for row in valid_trees
            ),
            "named_groups_total": sum(
                int(row["named_group_count"]) for row in valid_trees
            ),
            "pivots_mean": mean(int(row["pivot_count"]) for row in valid_trees),
            "pivots_total": sum(int(row["pivot_count"]) for row in valid_trees),
            "parent_child_edge_f1": None,
            "hierarchy_exact_match": None,
            "semantic_nesting_accuracy": None,
        },
        "cross_seed_supplement": {
            "factory_count": len(factory_rows),
            "all_36_assets_present_factory_count": sum(
                bool(row["all_36_assets_present"]) for row in factory_rows
            ),
            "aggregation": "unweighted macro mean over the 20 factories",
            **cross_seed_macro,
            "raw_signature": "sorted (parent, joint name, joint type, child) edges",
            "canonical_shape_signature": (
                "recursive rooted shape with joint types and visual/group node labels; "
                "semantic names and sibling order removed"
            ),
            "claim": (
                "supplementary topology stability only; variable-topology legality "
                "and semantic nesting correctness are not scored"
            ),
        },
        "unsupported_by_protocol": {
            "parent_child_edge_f1": "no independent hierarchy gold edge set",
            "hierarchy_exact_match": "no independent hierarchy gold tree",
            "semantic_nesting_accuracy": "no independent semantic nesting gold",
        },
    }

    frozen_manifest = {
        "protocol": protocol,
        "source_runtime": input_root.relative_to(EXP_ROOT).as_posix(),
        "source_manifest_sha256": sha256(input_root / "manifest.json"),
        "source_records_sha256": sha256(input_root / "records.json"),
        "protocol_sha256": sha256(protocol_path),
        "evaluator_sha256": sha256(Path(__file__).resolve()),
        "baseline_source_tree_sha256": source_manifest["baseline_source_tree_sha256"],
        "blender_sha256": source_manifest["blender_sha256"],
        "selection_case_count": requested_count,
        "selection": [
            {
                "factory": row["factory"],
                "seed": row["seed"],
                "baseline_status": row["baseline_status"],
                "case_dir": row["case_dir"],
                "urdf_path": row.get("urdf_path"),
                "urdf_sha256": row.get("urdf_sha256"),
                "recorded_package_sha256": row.get("recorded_package_sha256"),
            }
            for row in evaluated
        ],
    }

    table3 = summary["table3"]
    cross = summary["cross_seed_supplement"]
    report = [
        "# Infinite Mobility Table 3 Hierarchy",
        "",
        f"- Frozen cohort: 20 public factories x 36 seeds = {requested_count} cases",
        (
            f"- Generated assets: {len(valid)}/{requested_count}; "
            f"{requested_count - len(valid)} terminal timeout cases had no URDF"
        ),
        (
            f"- Valid Tree: {valid_tree_count}/{requested_count} = "
            f"{table3['valid_tree_rate_requested']:.3%} requested; "
            f"{valid_tree_count}/{len(valid)} = "
            f"{table3['valid_tree_rate_generated']:.3%} generated"
        ),
        (
            f"- Has Hierarchy: {has_hierarchy_count}/{requested_count} = "
            f"{table3['has_hierarchy_rate_requested']:.3%} requested; "
            f"{has_hierarchy_count}/{len(valid)} = "
            f"{table3['has_hierarchy_rate_generated']:.3%} generated"
        ),
        (
            f"- Semantic Depth: {table3['semantic_depth_mean']:.3f} mean "
            f"[{table3['semantic_depth_min']}, {table3['semantic_depth_max']}]"
        ),
        (
            f"- Named Groups: {table3['named_groups_mean']:.3f} mean; "
            f"{table3['named_groups_total']} total"
        ),
        (
            f"- Pivots: {table3['pivots_mean']:.3f} mean; "
            f"{table3['pivots_total']} total"
        ),
        "- Parent-Child Edge F1 / Exact Match / Semantic Nesting: N/A (no independent gold)",
        "",
        "## Cross-Seed supplementary stability",
        "",
        (
            f"- All 36 generated assets present: "
            f"{cross['all_36_assets_present_factory_count']}/20 factories"
        ),
        (
            f"- Raw edge-signature mode rate: "
            f"{cross['raw_edge_signature_mode_rate']:.3f} macro"
        ),
        (
            f"- Pairwise raw edge exact: "
            f"{cross['pairwise_raw_edge_exact_rate']:.3f} macro"
        ),
        (
            f"- Canonical shape mode rate: "
            f"{cross['canonical_shape_signature_mode_rate']:.3f} macro"
        ),
        (
            f"- Pairwise canonical shape exact: "
            f"{cross['pairwise_canonical_shape_exact_rate']:.3f} macro"
        ),
        "",
        "Cross-seed values are supplementary topology stability, not semantic correctness.",
        "This public-factory cohort is not a common-category matched comparison.",
    ]

    output_root.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Any] = {
        "frozen_manifest.json": frozen_manifest,
        "case_records.json": evaluated,
        "factory_cross_seed.json": factory_rows,
        "summary.json": summary,
    }
    for name, payload in outputs.items():
        (output_root / name).write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    (output_root / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"outputs={output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
