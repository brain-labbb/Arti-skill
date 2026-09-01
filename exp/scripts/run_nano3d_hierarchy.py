#!/usr/bin/env python3
"""Reproduce Nova3D's deterministic hierarchy axis on the frozen URDF pilot.

The paper evaluates four scene-graph properties: has tree, semantic depth,
named group nodes, and pivots.  This harness applies a documented, conservative
URDF-equivalent mapping and never mutates source assets.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import itertools
import json
from pathlib import Path
from statistics import mean
import xml.etree.ElementTree as ET


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = EXP_ROOT / "runtime/nano3d_asset_pilot/asset_manifest.jsonl"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/nano3d_hierarchy"
MOVABLE_TYPES = {"revolute", "continuous", "prismatic", "planar", "floating"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def mode_rate(values: list[object]) -> float | None:
    if not values:
        return None
    return Counter(values).most_common(1)[0][1] / len(values)


def load_manifest(path: Path) -> list[dict[str, object]]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(rows) != 33:
        raise ValueError(f"expected frozen 33-asset manifest, found {len(rows)}")
    if len({str(row["asset_id"]) for row in rows}) != len(rows):
        raise ValueError("duplicate asset_id in manifest")
    return rows


def parse_hierarchy(urdf_path: Path) -> dict[str, object]:
    xml_root = ET.parse(urdf_path).getroot()
    link_nodes = xml_root.findall("link")
    joint_nodes = xml_root.findall("joint")
    links = [node.attrib.get("name", "") for node in link_nodes]
    link_set = {name for name in links if name}
    visual_links = {
        node.attrib.get("name", "")
        for node in link_nodes
        if node.find("visual/geometry") is not None
    }
    collision_links = {
        node.attrib.get("name", "")
        for node in link_nodes
        if node.find("collision/geometry") is not None
    }

    adjacency: dict[str, list[str]] = {name: [] for name in link_set}
    children: set[str] = set()
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
        children.add(child)
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
        and len(edge_records) == max(0, len(link_set) - 1)
        and not malformed_edges
    )
    # Paper-equivalent group: named scene node that carries a subtree but no
    # renderable geometry. Collision-only links remain non-renderable groups.
    named_groups = sorted(
        name for name in link_set if adjacency.get(name) and name not in visual_links
    )
    return {
        "valid_tree": valid_tree,
        "root_names": roots,
        "link_count": len(link_set),
        "joint_count": len(joint_nodes),
        "edge_count": len(edge_records),
        "has_tree": bool(valid_tree and semantic_depth > 1),
        "semantic_depth": semantic_depth,
        "named_group_count": len(named_groups),
        "named_group_names": named_groups,
        "pivot_count": len(movable_joint_names),
        "pivot_names": sorted(movable_joint_names),
        "visual_link_count": len(visual_links),
        "collision_link_count": len(collision_links),
        "edge_signature": [list(edge) for edge in sorted(edge_records)],
        "malformed_edges": malformed_edges,
    }


def sibling_dirs(asset_path: Path) -> list[Path]:
    slug_dir = asset_path.parent
    return sorted(
        path for path in slug_dir.iterdir()
        if path.is_dir() and path.name.startswith("seed_") and (path / "model.urdf").is_file()
    )


def evaluate_selected(row: dict[str, object]) -> dict[str, object]:
    urdf = Path(str(row["model_urdf"]))
    current_hash = sha256(urdf)
    frozen_hash = str(row["model_urdf_sha256"])
    if current_hash != frozen_hash:
        raise ValueError(f"frozen URDF hash mismatch for {row['asset_id']}")
    return {
        "asset_id": row["asset_id"],
        "slug": row["slug"],
        "seed": row["seed"],
        "source": row["source"],
        "difficulty": row["difficulty"],
        "model_urdf": str(urdf),
        "model_urdf_sha256": current_hash,
        **parse_hierarchy(urdf),
    }


def evaluate_cohort(row: dict[str, object]) -> dict[str, object]:
    asset_path = Path(str(row["asset_path"]))
    seeds: list[dict[str, object]] = []
    signatures: list[tuple[tuple[str, str, str, str], ...]] = []
    depths: list[int] = []
    groups: list[int] = []
    pivots: list[int] = []
    for seed_dir in sibling_dirs(asset_path):
        try:
            metrics = parse_hierarchy(seed_dir / "model.urdf")
            signature = tuple(tuple(edge) for edge in metrics["edge_signature"])
            error = None
        except Exception as exc:  # noqa: BLE001
            metrics = {}
            signature = ()
            error = f"{type(exc).__name__}: {exc}"
        seed_row = {
            "seed_dir": seed_dir.name,
            "valid_tree": metrics.get("valid_tree", False),
            "semantic_depth": metrics.get("semantic_depth"),
            "named_group_count": metrics.get("named_group_count"),
            "pivot_count": metrics.get("pivot_count"),
            "edge_signature": metrics.get("edge_signature", []),
            "error": error,
        }
        seeds.append(seed_row)
        if metrics.get("valid_tree"):
            signatures.append(signature)
            depths.append(int(metrics["semantic_depth"]))
            groups.append(int(metrics["named_group_count"]))
            pivots.append(int(metrics["pivot_count"]))

    pairwise_exact = [left == right for left, right in itertools.combinations(signatures, 2)]
    return {
        "slug": row["slug"],
        "source": row["source"],
        "selected_asset_id": row["asset_id"],
        "cohort_size": len(seeds),
        "valid_tree_count": len(signatures),
        "raw_edge_signature_mode_rate": mode_rate(signatures),
        "pairwise_raw_edge_exact_rate": mean(pairwise_exact) if pairwise_exact else None,
        "semantic_depth_mode_rate": mode_rate(depths),
        "named_group_count_mode_rate": mode_rate(groups),
        "pivot_count_mode_rate": mode_rate(pivots),
        "seed_records": seeds,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    rows = load_manifest(args.manifest)
    selected = [evaluate_selected(row) for row in rows]
    cohorts = [evaluate_cohort(row) for row in rows]
    multi = [row for row in cohorts if int(row["cohort_size"]) > 1]

    summary = {
        "protocol": "nano3d_hierarchy_paper_equivalent_v1",
        "paper_source": "Nova3D arXiv:2607.22738v1, Section 7.2 and Table 10",
        "representation": "URDF-equivalent mapping; not the paper's native GLB evaluator",
        "mapping": {
            "scene_node": "named URDF link",
            "flat_depth": 1,
            "semantic_depth": "maximum named-link root-to-leaf path length; root counts as 1",
            "unnamed_wrapper_collapse": "not applicable: URDF links must be named",
            "named_group": "named link with children and no visual geometry",
            "pivot": "named non-fixed URDF joint",
            "has_tree": "valid tree and semantic_depth > 1",
        },
        "selected_assets": {
            "asset_count": len(selected),
            "valid_tree_count": sum(bool(row["valid_tree"]) for row in selected),
            "has_tree_count": sum(bool(row["has_tree"]) for row in selected),
            "has_tree_rate": mean(bool(row["has_tree"]) for row in selected),
            "semantic_depth_mean": mean(int(row["semantic_depth"]) for row in selected),
            "semantic_depth_min": min(int(row["semantic_depth"]) for row in selected),
            "semantic_depth_max": max(int(row["semantic_depth"]) for row in selected),
            "named_groups_mean": mean(int(row["named_group_count"]) for row in selected),
            "named_groups_total": sum(int(row["named_group_count"]) for row in selected),
            "pivots_mean": mean(int(row["pivot_count"]) for row in selected),
            "pivots_total": sum(int(row["pivot_count"]) for row in selected),
        },
        "cross_seed_supplement": {
            "cohort_count": len(cohorts),
            "multi_seed_cohort_count": len(multi),
            "single_seed_cohort_count": len(cohorts) - len(multi),
            "multi_seed_record_count": sum(int(row["cohort_size"]) for row in multi),
            "raw_edge_signature_mode_rate_mean": mean(
                float(row["raw_edge_signature_mode_rate"]) for row in multi
                if row["raw_edge_signature_mode_rate"] is not None
            ),
            "pairwise_raw_edge_exact_rate_mean": mean(
                float(row["pairwise_raw_edge_exact_rate"]) for row in multi
                if row["pairwise_raw_edge_exact_rate"] is not None
            ),
            "claim": "supplementary raw-topology stability only; variable-topology legality is not scored",
        },
        "unsupported_by_paper_protocol": {
            "parent_child_edge_f1": "paper explicitly does not score provable nesting correctness",
            "hierarchy_exact_match": "paper has no hierarchy gold comparison metric",
            "semantic_nesting_accuracy": "paper dropped the LLM-judge correctness rung",
            "formal_cross_seed_consistency": "paper reports no cross-seed hierarchy metric",
        },
    }

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "selected_asset_records.json").write_text(
        json.dumps(selected, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (args.output / "cross_seed_records.json").write_text(
        json.dumps(cohorts, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"outputs={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
