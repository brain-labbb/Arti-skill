#!/usr/bin/env python3
"""Shared representation-level hierarchy diagnostics for URDF assets."""

from __future__ import annotations

from collections import Counter, deque
import itertools
from math import log2
from pathlib import Path
from statistics import mean
from typing import Any, Iterable
import xml.etree.ElementTree as ET


MOVABLE_TYPES = {"revolute", "continuous", "prismatic", "planar", "floating"}


def _canonical_shape(
    node: str,
    adjacency: dict[str, list[tuple[str, str]]],
    visual_links: set[str],
) -> str:
    children = sorted(
        f"{joint_type}:{_canonical_shape(child, adjacency, visual_links)}"
        for child, joint_type in adjacency.get(node, [])
    )
    kind = "visual" if node in visual_links else "group"
    return f"{kind}[{','.join(children)}]"


def analyze_urdf(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected robot root, found {root.tag!r}")
    link_nodes = root.findall("link")
    joint_nodes = root.findall("joint")
    link_names = [node.attrib.get("name", "") for node in link_nodes]
    named_links = [name for name in link_names if name]
    links = set(named_links)
    unnamed_link_count = len(link_names) - len(named_links)
    duplicate_link_count = len(named_links) - len(links)
    visual_links = {
        node.attrib.get("name", "")
        for node in link_nodes
        if node.attrib.get("name") and node.find("visual/geometry") is not None
    }
    collision_links = {
        node.attrib.get("name", "")
        for node in link_nodes
        if node.attrib.get("name") and node.find("collision/geometry") is not None
    }

    adjacency: dict[str, list[tuple[str, str]]] = {name: [] for name in links}
    undirected: dict[str, set[str]] = {name: set() for name in links}
    indegree: Counter[str] = Counter()
    valid_edges: list[tuple[str, str, str, str]] = []
    malformed_edge_count = 0
    joint_type_counts: Counter[str] = Counter()
    for joint in joint_nodes:
        joint_name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "")
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent not in links or child not in links:
            malformed_edge_count += 1
            continue
        adjacency[parent].append((child, joint_type))
        undirected[parent].add(child)
        undirected[child].add(parent)
        indegree[child] += 1
        valid_edges.append((parent, joint_name, joint_type, child))
        joint_type_counts[joint_type] += 1

    roots = sorted(name for name in links if indegree[name] == 0)
    component_count = 0
    largest_component = 0
    remaining = set(links)
    while remaining:
        component_count += 1
        start = next(iter(remaining))
        queue = deque([start])
        component: set[str] = set()
        while queue:
            node = queue.popleft()
            if node in component:
                continue
            component.add(node)
            queue.extend(undirected[node] - component)
        remaining -= component
        largest_component = max(largest_component, len(component))

    cyclic = False
    visited: set[str] = set()
    active: set[str] = set()

    def visit(node: str) -> None:
        nonlocal cyclic
        if node in active:
            cyclic = True
            return
        if node in visited:
            return
        active.add(node)
        for child, _ in adjacency[node]:
            visit(child)
        active.remove(node)
        visited.add(node)

    for node in links:
        visit(node)

    max_depth = 0
    max_movable_depth = 0
    reached: set[str] = set()
    queue = deque((name, 1, 0) for name in roots)
    while queue:
        node, depth, movable_depth = queue.popleft()
        if node in reached:
            continue
        reached.add(node)
        max_depth = max(max_depth, depth)
        max_movable_depth = max(max_movable_depth, movable_depth)
        for child, joint_type in adjacency[node]:
            queue.append(
                (child, depth + 1, movable_depth + int(joint_type in MOVABLE_TYPES))
            )

    valid_tree = (
        len(links) > 0
        and unnamed_link_count == 0
        and duplicate_link_count == 0
        and len(roots) == 1
        and component_count == 1
        and not cyclic
        and len(valid_edges) == len(links) - 1
        and malformed_edge_count == 0
    )
    leaves = [name for name in links if not adjacency[name]]
    internal = [name for name in links if adjacency[name]]
    branching = [name for name in links if len(adjacency[name]) >= 2]
    groups = [name for name in internal if name not in visual_links]
    movable_edge_count = sum(
        count for joint_type, count in joint_type_counts.items() if joint_type in MOVABLE_TYPES
    )
    fixed_edge_count = joint_type_counts.get("fixed", 0)
    node_count = len(links)
    edge_count = len(valid_edges)
    canonical_signature = (
        _canonical_shape(roots[0], adjacency, visual_links) if valid_tree else None
    )
    return {
        "valid_tree": valid_tree,
        "node_count": node_count,
        "edge_count": edge_count,
        "root_count": len(roots),
        "component_count": component_count,
        "largest_component_node_rate": largest_component / node_count if node_count else None,
        "malformed_edge_count": malformed_edge_count,
        "unnamed_link_count": unnamed_link_count,
        "duplicate_link_count": duplicate_link_count,
        "cyclic": cyclic,
        "multi_parent_node_count": sum(count > 1 for count in indegree.values()),
        "valid_joint_endpoint_rate": (
            edge_count / len(joint_nodes) if joint_nodes else 1.0
        ),
        "leaf_count": len(leaves),
        "leaf_ratio": len(leaves) / node_count if node_count else None,
        "internal_node_count": len(internal),
        "branching_node_count": len(branching),
        "branching_node_ratio": len(branching) / node_count if node_count else None,
        "mean_internal_out_degree": (
            mean(len(adjacency[name]) for name in internal) if internal else 0.0
        ),
        "max_out_degree": max((len(children) for children in adjacency.values()), default=0),
        "semantic_depth": max_depth,
        "movable_depth": max_movable_depth,
        "fixed_edge_count": fixed_edge_count,
        "movable_edge_count": movable_edge_count,
        "other_edge_count": edge_count - fixed_edge_count - movable_edge_count,
        "movable_edge_ratio": movable_edge_count / edge_count if edge_count else 0.0,
        "visual_link_count": len(visual_links),
        "visual_link_ratio": len(visual_links) / node_count if node_count else None,
        "collision_link_count": len(collision_links),
        "collision_link_ratio": len(collision_links) / node_count if node_count else None,
        "group_node_count": len(groups),
        "group_node_ratio": len(groups) / node_count if node_count else None,
        "joint_type_counts": dict(sorted(joint_type_counts.items())),
        "raw_edge_signature": [list(edge) for edge in sorted(valid_edges)],
        "canonical_topology_signature": canonical_signature,
    }


def _mode_rate(values: list[str]) -> float | None:
    return Counter(values).most_common(1)[0][1] / len(values) if values else None


def _pairwise_exact(values: list[str]) -> float | None:
    pairs = [left == right for left, right in itertools.combinations(values, 2)]
    return mean(pairs) if pairs else None


def _normalized_entropy(values: list[str]) -> float | None:
    if len(values) <= 1:
        return None
    counts = Counter(values)
    entropy = -sum((count / len(values)) * log2(count / len(values)) for count in counts.values())
    return entropy / log2(len(values))


def aggregate(records: Iterable[dict[str, Any]], requested_count: int) -> dict[str, Any]:
    rows = list(records)
    valid = [row for row in rows if row.get("valid_tree")]
    numeric_fields = [
        "node_count",
        "edge_count",
        "leaf_count",
        "leaf_ratio",
        "internal_node_count",
        "branching_node_count",
        "branching_node_ratio",
        "mean_internal_out_degree",
        "max_out_degree",
        "semantic_depth",
        "movable_depth",
        "fixed_edge_count",
        "movable_edge_count",
        "movable_edge_ratio",
        "visual_link_count",
        "visual_link_ratio",
        "collision_link_count",
        "collision_link_ratio",
        "group_node_count",
        "group_node_ratio",
        "component_count",
        "root_count",
    ]
    metrics: dict[str, Any] = {
        "requested_count": requested_count,
        "evaluated_count": len(rows),
        "valid_tree_count": len(valid),
        "valid_tree_rate_requested": len(valid) / requested_count if requested_count else None,
        "single_root_count": sum(row["root_count"] == 1 for row in rows),
        "root_defect_count": sum(row["root_count"] != 1 for row in rows),
        "connected_count": sum(row["component_count"] == 1 for row in rows),
        "component_defect_count": sum(row["component_count"] != 1 for row in rows),
        "cycle_defect_count": sum(bool(row["cyclic"]) for row in rows),
        "malformed_edge_asset_count": sum(row["malformed_edge_count"] > 0 for row in rows),
        "multi_parent_asset_count": sum(row["multi_parent_node_count"] > 0 for row in rows),
        "unnamed_or_duplicate_link_asset_count": sum(
            row["unnamed_link_count"] > 0 or row["duplicate_link_count"] > 0
            for row in rows
        ),
        "largest_component_node_rate_mean_evaluated": (
            mean(float(row["largest_component_node_rate"]) for row in rows)
            if rows
            else None
        ),
        "valid_joint_endpoint_rate_mean_evaluated": (
            mean(float(row["valid_joint_endpoint_rate"]) for row in rows)
            if rows
            else None
        ),
    }
    for field in numeric_fields:
        values = [float(row[field]) for row in valid if row.get(field) is not None]
        metrics[f"{field}_mean"] = mean(values) if values else None
        metrics[f"{field}_total"] = sum(values) if values else None
    return metrics


def topology_consistency(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    rows = [row for row in records if row.get("canonical_topology_signature")]
    signatures = [str(row["canonical_topology_signature"]) for row in rows]
    return {
        "sample_count": len(signatures),
        "unique_signature_count": len(set(signatures)),
        "unique_signature_rate": len(set(signatures)) / len(signatures) if signatures else None,
        "mode_rate": _mode_rate(signatures),
        "pairwise_exact_rate": _pairwise_exact(signatures),
        "normalized_entropy": _normalized_entropy(signatures),
        "claim": "name-free rooted topology diversity only; not semantic correctness",
    }
